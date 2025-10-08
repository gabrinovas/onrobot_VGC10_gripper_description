#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_control.msg import OnRobotRGInput


class OnRobotRGStatusListener(Node):
    def __init__(self):
        super().__init__('OnRobotRGStatusListener')
        self.subscription = self.create_subscription(
            OnRobotRGInput,
            'OnRobotRGInput',
            self.print_status,
            10)
        
    def print_status(self, msg):
        self.get_logger().info(self.status_interpreter(msg))
        
    def status_interpreter(self, status):
        output = '\n-----\nOnRobot RG status interpreter\n-----\n'

        # gFOF
        output += f'gFOF = {status.g_fof}: '
        output += f'Current fingertip offset: {status.g_fof / 10.0} mm\n'

        # gGWD
        output += f'gGWD = {status.g_gwd}: '
        output += f'Current width between fingers (w/o offset): {status.g_gwd / 10.0} mm\n'

        # gSTA
        output += f'gSTA = {status.g_sta}: '
        gSTA16bit = format(status.g_sta, '016b')
        output += f'(gSTA (16 bit) = {gSTA16bit}), Current states: '
        
        if int(gSTA16bit[-1]):
            output += ' A motion is ongoing so new commands are not accepted.'
        if int(gSTA16bit[-2]):
            output += ' An internal- or external grip is detected.'
        if int(gSTA16bit[-3]):
            output += ' Safety switch 1 is pushed.'
        if int(gSTA16bit[-4]):
            output += ' Safety circuit 1 is activated so the gripper cannot move.'
        if int(gSTA16bit[-5]):
            output += ' Safety switch 2 is pushed.'
        if int(gSTA16bit[-6]):
            output += ' Safety circuit 2 is activated so the gripper cannot move.'
        if int(gSTA16bit[-7]):
            output += ' Any of the safety switch is pushed.'

        # gWDF
        output += f'\ngWDF = {status.g_wdf}: '
        output += f'Current width between fingers (w offset): {status.g_wdf / 10.0} mm\n'

        return output


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGStatusListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()