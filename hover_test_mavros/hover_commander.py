from geometry_msgs.msg import Point

import rclpy

from rclpy.node import Node

from std_msgs.msg import Float32, Int32


class HoverCommander(Node):

    def __init__(self):

        super().__init__('hover_commander')

        self.publisher = self.create_publisher(
            Int32,
            '/hover_test/command',
            10
        )

        self.yaw_publisher = self.create_publisher(
            Float32,
            '/hover_test/yaw_command',
            10
        )

        self.position_publisher = self.create_publisher(
            Point,
            '/hover_test/position_command',
            10
        )

    # ================================================================
    # Blocking input loop
    #
    # Runs in its own terminal/process, separate from
    # hover_control's node (which prints frequent status logs). That
    # keeps this prompt from being scrolled away by log spam.
    # ================================================================

    def run(self):

        # Numbers are left out of 1/3/4's hints on purpose - they're
        # hover_control's hover_altitude_m/horizontal_step_m
        # parameters, which can be overridden at launch
        # (--ros-args -p ...) and this node has no way to know what
        # value the other process was actually started with.
        # Frames (MAVROS version): +X/+Y and (x, y) are mocap axes and
        # absolute mocap coordinates; heading 0 = mocap +X,
        # counter-clockwise positive.
        prompt = (
            '\n[Command] 1=takeoff to hover  2=land  3=+X step  '
            '4=+Y step  5 <deg>=heading (0=mocap +X, CCW+)  '
            '6 <x> <y>=fly to mocap (x, y) (m)  > '
        )

        while rclpy.ok():

            try:
                line = input(prompt).strip()
            except EOFError:
                break

            tokens = line.split()

            if tokens and tokens[0] == '5':

                if len(tokens) != 2:

                    print('Usage: 5 <deg>, e.g. "5 90"')

                    continue

                try:
                    deg = float(tokens[1])
                except ValueError:

                    print(f'Invalid heading "{tokens[1]}", expected a number')

                    continue

                msg = Float32()

                msg.data = deg

                self.yaw_publisher.publish(msg)

                print(f'Sent heading command {deg:.1f} deg')

                continue

            if tokens and tokens[0] == '6':

                if len(tokens) != 3:

                    print('Usage: 6 <x> <y> (mocap meters), e.g. "6 1.0 0.5"')

                    continue

                try:
                    x = float(tokens[1])
                    y = float(tokens[2])
                except ValueError:

                    print(
                        f'Invalid position "{tokens[1]} {tokens[2]}", '
                        'expected two numbers'
                    )

                    continue

                msg = Point()

                msg.x = x
                msg.y = y
                msg.z = 0.0

                self.position_publisher.publish(msg)

                print(f'Sent position command x={x:.2f}m, y={y:.2f}m')

                continue

            if line not in ('1', '2', '3', '4'):

                print(
                    f'Unknown command "{line}", use 1, 2, 3, 4, '
                    '5 <deg> or 6 <x> <y>'
                )

                continue

            msg = Int32()

            msg.data = int(line)

            self.publisher.publish(msg)

            print(f'Sent command {line}')


def main(args=None):

    rclpy.init(args=args)

    node = HoverCommander()

    try:

        node.run()

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()

        rclpy.shutdown()


if __name__ == '__main__':

    main()
