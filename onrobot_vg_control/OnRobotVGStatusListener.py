#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_control.msg import OnRobotVGInput


class OnRobotVGStatusListener(Node):
    def __init__(self):
        super().__init__('OnRobotVGStatusListener')
        self.subscription = self.create_subscription(
            OnRobotVGInput,
            'OnRobotVGInput',
            self.print_status,
            10)
        
    def print_status(self, msg):
        self.get_logger().info(self.status_interpreter(msg))
        
    def status_interpreter(self, status):
        output = '\n-----\nOnRobot VG status interpreter\n-----\n'

        # gVCA - Channel A vacuum
        output += f'gVCA = {status.g_vca}: '
        output += f'Current vacuum value on Channel A: {status.g_vca} (0-1000 scale)\n'

        # gVCB - Channel B vacuum
        output += f'gVCB = {status.g_vcb}: '
        output += f'Current vacuum value on Channel B: {status.g_vcb} (0-1000 scale)\n'

        # Interpretation for both VG10 and VGC10
        output += '\nInterpretation:\n'
        output += '- VG10: Uses both channels A and B\n'
        output += '- VGC10: Typically uses channel A only\n'
        output += '- Values > 800 indicate good vacuum\n'
        output += '- Values < 200 indicate poor/no vacuum\n'

        return output


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotVGStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()