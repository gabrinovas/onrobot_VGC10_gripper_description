#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_control.msg import OnRobotVGOutput


class OnRobotVGSimpleController(Node):
    def __init__(self):
        super().__init__('OnRobotVGSimpleController')
        
        self.pub = self.create_publisher(OnRobotVGOutput, 'OnRobotVGOutput', 10)
        self.command = OnRobotVGOutput()
        
        self.timer = self.create_timer(0.1, self.publish_loop)

    def ask_for_command(self):
        currentCommand = 'Simple OnRobot VG Controller\n-----\nCurrent command:'
        currentCommand += f' rMCA = {self.command.r_mca:#06x}'
        currentCommand += f', rVCA = {self.command.r_vca}'
        currentCommand += f', rMCB = {self.command.r_mcb:#06x}'
        currentCommand += f', rVCB = {self.command.r_vcb}'

        self.get_logger().info(currentCommand)

        strAskForCommand = '-----\nAvailable commands (VG10 & VGC10)\n\n'
        strAskForCommand += 'g: Turn on all channels\n'
        strAskForCommand += 'r: Turn off all channels\n'
        strAskForCommand += 'ga: Turn on channel A only\n'
        strAskForCommand += 'ra: Turn off channel A only\n'
        strAskForCommand += 'gb: Turn on channel B only\n'
        strAskForCommand += 'rb: Turn off channel B only\n'
        strAskForCommand += 'ia: Idle channel A\n'
        strAskForCommand += 'ib: Idle channel B\n'
        strAskForCommand += '(0-255): Set vacuum power for all channels\n'
        strAskForCommand += '-->'

        return input(strAskForCommand)

    def gen_command(self, char, command):
        if char == 'g':
            command.r_mca = 0x0100
            command.r_vca = 255
            command.r_mcb = 0x0100
            command.r_vcb = 255
        elif char == 'r':
            command.r_mca = 0x0000
            command.r_vca = 0
            command.r_mcb = 0x0000
            command.r_vcb = 0
        elif char == 'ga':
            command.r_mca = 0x0100
            command.r_vca = 255
        elif char == 'ra':
            command.r_mca = 0x0000
            command.r_vca = 0
        elif char == 'gb':
            command.r_mcb = 0x0100
            command.r_vcb = 255
        elif char == 'rb':
            command.r_mcb = 0x0000
            command.r_vcb = 0
        elif char == 'ia':
            command.r_mca = 0x0200
        elif char == 'ib':
            command.r_mcb = 0x0200
        else:
            try:
                vacuum_level = min(255, int(char))
                if vacuum_level == 0:
                    command.r_mca = 0x0000
                    command.r_vca = 0
                    command.r_mcb = 0x0000
                    command.r_vcb = 0
                else:
                    command.r_mca = 0x0100
                    command.r_vca = vacuum_level
                    command.r_mcb = 0x0100
                    command.r_vcb = vacuum_level
            except ValueError:
                self.get_logger().warn(f"Unknown VG command: {char}")

        return command

    def publish_loop(self):
        try:
            char = self.ask_for_command()
            self.command = self.gen_command(char, self.command)
            self.pub.publish(self.command)
        except EOFError:
            self.get_logger().info("Shutting down VG controller...")
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotVGSimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()