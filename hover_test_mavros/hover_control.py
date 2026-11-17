"""Single UAV hover for ArduPilot over MAVROS (port of hover_test)."""

import math
import time

from geometry_msgs.msg import Point, PoseStamped

from mavros_msgs.msg import PositionTarget, State
from mavros_msgs.srv import CommandBool, CommandTOL, SetMode

import rclpy

from rclpy.node import Node

from rclpy.qos import qos_profile_sensor_data

from rclpy.signals import SignalHandlerOptions

from std_msgs.msg import Float32, Int32


# ====================================================================
# Frames
#
# Everything this node reads and writes on the ROS side is ENU with
# z up, in the mocap frame: /mavros/local_position/pose x/y are the
# Motive x/y axes (origin = mocap origin, NOT the takeoff point), and
# yaw is 0 along mocap +X, counter-clockwise positive. MAVROS converts
# to/from ArduPilot's NED internally - never convert here.
# ====================================================================

# Position-only target: ignore velocity, acceleration and yaw rate.
POSITION_MASK = (
    PositionTarget.IGNORE_VX
    | PositionTarget.IGNORE_VY
    | PositionTarget.IGNORE_VZ
    | PositionTarget.IGNORE_AFX
    | PositionTarget.IGNORE_AFY
    | PositionTarget.IGNORE_AFZ
    | PositionTarget.IGNORE_YAW_RATE
)

# Seconds between repeated requests of the same service while waiting
# for the vehicle state to change.
REQUEST_RETRY_S = 1.0

# Give up a takeoff after this many failed arm / takeoff requests.
MAX_ATTEMPTS = 5


def normalize_angle(angle):

    # Wraps to -PI..+PI so angle differences always take the
    # shortest way around (350 deg vs 10 deg is a 20 deg step).
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):

    # ENU yaw (rotation about z, 0 along +X, counter-clockwise positive).
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class SingleUAVHoverMavros(Node):

    def __init__(self):

        super().__init__('hover_test_mavros_control')

        # ============================================================
        # Parameters
        # ============================================================

        self.declare_parameter('mavros_ns', '/mavros')
        self.declare_parameter('hover_altitude_m', 0.5)
        self.declare_parameter('horizontal_speed_mps', 0.2)
        self.declare_parameter('horizontal_step_m', 0.5)
        self.declare_parameter('yaw_rate_dps', 30.0)
        self.declare_parameter('max_goal_distance_m', 3.0)
        self.declare_parameter('hover_reached_tol_m', 0.1)

        self.mavros_ns = self.get_parameter(
            'mavros_ns'
        ).get_parameter_value().string_value.rstrip('/')

        self.hover_altitude = abs(self.get_parameter(
            'hover_altitude_m'
        ).get_parameter_value().double_value)

        self.horizontal_speed = self.get_parameter(
            'horizontal_speed_mps'
        ).get_parameter_value().double_value

        self.horizontal_step = self.get_parameter(
            'horizontal_step_m'
        ).get_parameter_value().double_value

        self.yaw_rate = math.radians(
            self.get_parameter(
                'yaw_rate_dps'
            ).get_parameter_value().double_value
        )

        self.max_goal_distance = self.get_parameter(
            'max_goal_distance_m'
        ).get_parameter_value().double_value

        self.hover_tol = self.get_parameter(
            'hover_reached_tol_m'
        ).get_parameter_value().double_value

        self.get_logger().info(
            'Single UAV Hover (MAVROS / ArduPilot) started! '
            f'mavros_ns={self.mavros_ns}, '
            f'hover_altitude={self.hover_altitude:.2f}m'
        )

        ns = self.mavros_ns

        # ============================================================
        # MAVROS interfaces
        #
        # Subscriptions use the sensor-data profile (BEST_EFFORT):
        # it matches both RELIABLE and BEST_EFFORT publishers.
        # ============================================================

        self.mav_state = None
        self.pose = None

        self.create_subscription(
            State,
            f'{ns}/state',
            self.state_callback,
            qos_profile_sensor_data
        )

        self.create_subscription(
            PoseStamped,
            f'{ns}/local_position/pose',
            self.pose_callback,
            qos_profile_sensor_data
        )

        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            f'{ns}/setpoint_raw/local',
            10
        )

        self.set_mode_cli = self.create_client(SetMode, f'{ns}/set_mode')
        self.arming_cli = self.create_client(CommandBool, f'{ns}/cmd/arming')
        self.takeoff_cli = self.create_client(CommandTOL, f'{ns}/cmd/takeoff')

        # ============================================================
        # Commands from hover_commander (same topics as hover_test)
        # ============================================================

        self.create_subscription(
            Int32, '/hover_test/command', self.command_callback, 10
        )

        self.create_subscription(
            Float32, '/hover_test/yaw_command', self.yaw_command_callback, 10
        )

        self.create_subscription(
            Point,
            '/hover_test/position_command',
            self.position_command_callback,
            10
        )

        # ============================================================
        # State machine
        #
        # idle       - on the ground, nothing sent to the vehicle
        # taking_off - GUIDED -> arm -> MAV_CMD_NAV_TAKEOFF, wait for
        #              hover altitude (ArduPilot Copter will not leave
        #              the ground on a GUIDED position target alone)
        # hovering   - streams position targets, ramps toward goal
        # landing    - LAND mode; ArduPilot descends and disarms on
        #              touchdown, back to idle once disarmed
        # ============================================================

        self.state = 'idle'

        # Takeoff point (mocap frame), set on command 1.
        self.home_x = 0.0
        self.home_y = 0.0
        self.home_z = 0.0
        self.hover_z = 0.0

        self.target_x = 0.0
        self.target_y = 0.0
        self.goal_x = 0.0
        self.goal_y = 0.0

        # Heading in radians (ENU). NaN = do not control yaw
        # (IGNORE_YAW): ArduPilot keeps its current heading.
        self.yaw_target = float('nan')
        self.yaw_setpoint = float('nan')

        self.guided_seen = False
        self.land_seen = False
        self.takeoff_sent = False
        self.arm_attempts = 0
        self.takeoff_attempts = 0
        self.takeoff_started = 0.0

        # Service bookkeeping: one in-flight call per service, and
        # a minimum interval between repeats.
        self.pending = {}
        self.last_request = {}

        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.timer_callback)

        self.last_status_print = 0.0

    # ================================================================
    # Callbacks
    # ================================================================

    def state_callback(self, msg):

        self.mav_state = msg

    def pose_callback(self, msg):

        self.pose = msg

    def current_yaw(self):

        return yaw_from_quaternion(self.pose.pose.orientation)

    def yaw_command_callback(self, msg):

        self.yaw_target = normalize_angle(math.radians(msg.data))

        self.get_logger().info(
            f'Command 5 received: heading set to {msg.data:.1f} deg '
            '(0 = mocap +X, counter-clockwise positive); applied while '
            'hovering'
        )

    def position_command_callback(self, msg):

        if self.state != 'hovering':

            self.get_logger().info(
                f'Command 6 ignored: not hovering (state="{self.state}")'
            )

            return

        if not self.goal_is_safe(msg.x, msg.y):
            return

        self.goal_x = msg.x
        self.goal_y = msg.y

        self.get_logger().info(
            'Command 6 received: flying to mocap '
            f'x={self.goal_x:.2f}m, y={self.goal_y:.2f}m'
        )

    def goal_is_safe(self, x, y):

        if self.pose is None:
            return False

        p = self.pose.pose.position
        dist = math.hypot(x - p.x, y - p.y)

        if dist > self.max_goal_distance:

            self.get_logger().warn(
                f'Goal ({x:.2f}, {y:.2f}) is {dist:.2f}m away, more than '
                f'max_goal_distance_m={self.max_goal_distance:.2f}m - '
                'ignored'
            )

            return False

        return True

    def command_callback(self, msg):

        command = msg.data

        if command == 1:

            if self.state != 'idle':

                self.get_logger().info(
                    f'Command 1 ignored: already in state "{self.state}"'
                )

                return

            if (
                self.pose is None
                or self.mav_state is None
                or not self.mav_state.connected
            ):

                self.get_logger().warn(
                    'Command 1 ignored: no MAVROS connection or no '
                    'local position yet'
                )

                return

            p = self.pose.pose.position

            self.home_x = p.x
            self.home_y = p.y
            self.home_z = p.z
            self.hover_z = p.z + self.hover_altitude

            self.target_x = self.goal_x = p.x
            self.target_y = self.goal_y = p.y

            self.guided_seen = False
            self.takeoff_sent = False
            self.arm_attempts = 0
            self.takeoff_attempts = 0
            self.takeoff_started = time.monotonic()
            self.yaw_setpoint = float('nan')

            self.state = 'taking_off'

            self.get_logger().info(
                'Command 1 received: taking off from mocap '
                f'({p.x:.2f}, {p.y:.2f}, {p.z:.2f}) to '
                f'{self.hover_altitude:.2f}m above it'
            )

        elif command == 2:

            if self.state in ('taking_off', 'hovering'):

                self.start_landing()

                self.get_logger().info('Command 2 received: landing')

            else:

                self.get_logger().info(
                    f'Command 2 ignored: not flying (state="{self.state}")'
                )

        elif command in (3, 4):

            if self.state != 'hovering':

                self.get_logger().info(
                    f'Command {command} ignored: not hovering '
                    f'(state="{self.state}")'
                )

                return

            new_x = self.goal_x + (self.horizontal_step if command == 3 else 0.0)
            new_y = self.goal_y + (self.horizontal_step if command == 4 else 0.0)

            if not self.goal_is_safe(new_x, new_y):
                return

            self.goal_x = new_x
            self.goal_y = new_y

            self.get_logger().info(
                f'Command {command} received: moving to mocap '
                f'x={self.goal_x:.2f}m, y={self.goal_y:.2f}m'
            )

        else:

            self.get_logger().info(
                f'Unknown command "{command}", use 1 (hover), 2 (land), '
                '3 (+X step) or 4 (+Y step)'
            )

    # ================================================================
    # Service helpers
    # ================================================================

    def request(self, name, client, req, on_done=None):

        # Fire an async service call unless one is still in flight or
        # the same call was made less than REQUEST_RETRY_S ago. Never
        # blocks the executor.
        now = time.monotonic()

        if self.pending.get(name) is not None:
            return

        if now - self.last_request.get(name, -1e9) < REQUEST_RETRY_S:
            return

        if not client.service_is_ready():

            self.get_logger().warn(
                f'Service for "{name}" not available yet',
                throttle_duration_sec=2.0
            )

            return

        self.last_request[name] = now

        future = client.call_async(req)

        self.pending[name] = future

        def done(fut):

            self.pending[name] = None

            if on_done is not None:
                on_done(fut)

        future.add_done_callback(done)

    def request_mode(self, mode):

        def done(fut):

            res = fut.result()

            if res is None or not res.mode_sent:
                self.get_logger().warn(f'SetMode {mode} was not sent')

        self.request(
            f'mode_{mode}',
            self.set_mode_cli,
            SetMode.Request(custom_mode=mode),
            done
        )

    def request_arm(self, arm):

        def done(fut):

            res = fut.result()
            ok = res is not None and res.success

            self.get_logger().info(
                f'{"ARM" if arm else "DISARM"} '
                f'{"accepted" if ok else "rejected"}'
                + ('' if res is None else f' (result={res.result})')
            )

            if arm and not ok:
                self.arm_attempts += 1

        self.request(
            'arm' if arm else 'disarm',
            self.arming_cli,
            CommandBool.Request(value=arm),
            done
        )

    def request_takeoff(self):

        def done(fut):

            res = fut.result()
            ok = res is not None and res.success

            self.get_logger().info(
                f'TAKEOFF {"accepted" if ok else "rejected"}'
                + ('' if res is None else f' (result={res.result})')
            )

            if ok:
                self.takeoff_sent = True
            else:
                self.takeoff_attempts += 1

        self.request(
            'takeoff',
            self.takeoff_cli,
            CommandTOL.Request(altitude=float(self.hover_altitude)),
            done
        )

    # ================================================================
    # Vehicle state helpers
    # ================================================================

    def mode(self):

        return '' if self.mav_state is None else self.mav_state.mode

    def is_armed(self):

        return self.mav_state is not None and self.mav_state.armed

    # ================================================================
    # Setpoint
    # ================================================================

    def update_yaw_setpoint(self):

        if math.isnan(self.yaw_target):

            self.yaw_setpoint = float('nan')

            return

        if math.isnan(self.yaw_setpoint):

            # Start the ramp from the vehicle's actual heading.
            self.yaw_setpoint = (
                self.current_yaw() if self.pose is not None
                else self.yaw_target
            )

        diff = normalize_angle(self.yaw_target - self.yaw_setpoint)

        max_step = self.yaw_rate * self.dt

        if abs(diff) <= max_step:
            self.yaw_setpoint = self.yaw_target
        else:
            self.yaw_setpoint = normalize_angle(
                self.yaw_setpoint + math.copysign(max_step, diff)
            )

    def publish_setpoint(self, x, y, z):

        msg = PositionTarget()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'

        # FRAME_LOCAL_NED: MAVROS converts these ENU values to NED.
        msg.coordinate_frame = PositionTarget.FRAME_LOCAL_NED

        msg.type_mask = POSITION_MASK

        msg.position.x = float(x)
        msg.position.y = float(y)
        msg.position.z = float(z)

        if math.isnan(self.yaw_setpoint):
            msg.type_mask |= PositionTarget.IGNORE_YAW
        else:
            msg.yaw = float(self.yaw_setpoint)

        self.setpoint_pub.publish(msg)

    # ================================================================
    # Status
    # ================================================================

    def print_status(self):

        if self.pose is None:
            return

        p = self.pose.pose.position

        yaw_sp = (
            'free' if math.isnan(self.yaw_setpoint)
            else f'{math.degrees(self.yaw_setpoint):.1f}deg'
        )

        self.get_logger().info(
            f'state={self.state}, mode={self.mode()}, '
            f'ARMED={self.is_armed()}, '
            f'x={p.x:.2f} (target={self.target_x:.2f}), '
            f'y={p.y:.2f} (target={self.target_y:.2f}), '
            f'z={p.z:.2f} (hover={self.hover_z:.2f}), '
            f'yaw={math.degrees(self.current_yaw()):.1f}deg '
            f'(target={yaw_sp})'
        )

    # ================================================================
    # Main state machine
    # ================================================================

    def start_landing(self):

        self.land_seen = False

        self.state = 'landing'

    def abort_takeoff(self, reason):

        self.get_logger().error(f'Takeoff aborted: {reason}')

        if self.is_armed():
            self.start_landing()
        else:
            self.state = 'idle'

    def pilot_override(self):

        # Someone (RC transmitter / GCS) switched the vehicle out of
        # the mode we put it in: stop commanding it.
        mode = self.mode()

        if mode == 'LAND':

            self.get_logger().warn('Mode changed to LAND externally')

            self.start_landing()

            self.land_seen = True

        else:

            self.get_logger().warn(
                f'Mode changed to {mode} externally - stop commanding '
                '(state -> idle)'
            )

            self.state = 'idle'

    def timer_callback(self):

        if self.state == 'idle':
            return

        if self.pose is None or self.mav_state is None:

            self.get_logger().info(
                'Waiting for MAVROS state/position...',
                throttle_duration_sec=2.0
            )

            return

        now = time.monotonic()

        if now - self.last_status_print > 5.0:

            self.print_status()

            self.last_status_print = now

        p = self.pose.pose.position

        if self.state == 'taking_off':

            if self.mode() != 'GUIDED':

                if self.guided_seen:
                    self.pilot_override()
                    return

                if now - self.takeoff_started > 5.0:
                    self.abort_takeoff(
                        f'vehicle did not enter GUIDED (mode={self.mode()}) '
                        '- check /mavros/statustext/recv'
                    )
                    return

                self.request_mode('GUIDED')

                return

            self.guided_seen = True

            if not self.is_armed():

                if self.takeoff_sent:
                    self.abort_takeoff('vehicle disarmed during takeoff')
                    return

                if self.arm_attempts >= MAX_ATTEMPTS:
                    self.abort_takeoff(
                        'arming rejected - check /mavros/statustext/recv '
                        'for PreArm messages'
                    )
                    return

                self.request_arm(True)

                return

            if not self.takeoff_sent:

                if self.takeoff_attempts >= MAX_ATTEMPTS:
                    self.abort_takeoff('takeoff command rejected')
                    return

                # Right after arming: ArduPilot auto-disarms after
                # DISARM_DELAY if the vehicle does not take off.
                self.request_takeoff()

                return

            if p.z >= self.hover_z - self.hover_tol:

                self.state = 'hovering'

                self.get_logger().info('=' * 40)
                self.get_logger().info(
                    f'HOVER REACHED at {p.z - self.home_z:.2f}m above '
                    'the takeoff point'
                )
                self.get_logger().info(
                    'Commands: 2 = land, 3/4 = +X/+Y step, '
                    '5 <deg> = heading, 6 <x> <y> = fly to mocap (x, y)'
                )
                self.get_logger().info('=' * 40)

        elif self.state == 'hovering':

            if self.mode() != 'GUIDED':
                self.pilot_override()
                return

            if not self.is_armed():
                self.get_logger().warn('Vehicle disarmed while hovering')
                self.state = 'idle'
                return

            self.target_x = self.ramp(self.target_x, self.goal_x)
            self.target_y = self.ramp(self.target_y, self.goal_y)

            self.update_yaw_setpoint()

            self.publish_setpoint(self.target_x, self.target_y, self.hover_z)

        elif self.state == 'landing':

            if not self.is_armed():

                self.get_logger().info('Landed and disarmed')

                self.state = 'idle'

                self.yaw_setpoint = float('nan')

                return

            if self.mode() == 'LAND':
                self.land_seen = True
                return

            if self.land_seen:
                # LAND was active, then someone switched away from it.
                self.pilot_override()
                return

            self.request_mode('LAND')

    def ramp(self, current, goal):

        step = self.horizontal_speed * self.dt

        if current < goal:
            return min(current + step, goal)

        if current > goal:
            return max(current - step, goal)

        return current

    # ================================================================
    # Ctrl+C: switch to LAND before exiting. ArduPilot finishes the
    # landing and disarms by itself even after this node is gone.
    # ================================================================

    def emergency_land(self):

        if self.state == 'idle' or not self.is_armed():
            return

        self.get_logger().info('Shutdown requested: switching to LAND')

        deadline = time.monotonic() + 5.0

        while rclpy.ok() and time.monotonic() < deadline:

            if self.mode() == 'LAND' or not self.is_armed():

                self.get_logger().info(
                    'LAND active - ArduPilot will land and disarm'
                )

                return

            self.request_mode('LAND')

            rclpy.spin_once(self, timeout_sec=self.dt)

        self.get_logger().error(
            'Could not confirm LAND mode - take over with the RC '
            'transmitter!'
        )


def main(args=None):

    rclpy.init(
        args=args,
        signal_handler_options=SignalHandlerOptions.NO
    )

    node = SingleUAVHoverMavros()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        try:

            node.emergency_land()

        except KeyboardInterrupt:

            node.get_logger().warn(
                'Landing request interrupted, shutting down immediately'
            )

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
