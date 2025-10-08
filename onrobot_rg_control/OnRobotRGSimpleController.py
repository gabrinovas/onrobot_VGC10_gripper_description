#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_control.msg import OnRobotRGOutput


class OnRobotRGSimpleController(Node):
    def __init__(self):
        super().__init__('OnRobotRGSimpleController')
        
        self.declare_parameter('gripper', 'rg6')
        self.gtype = self.get_parameter('gripper').value
        
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 10)
        self.command = OnRobotRGOutput()
        
        self.timer = self.create_timer(0.1, self.publish_loop)

    def ask_for_command(self):
        currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
        currentCommand += f' rGFR = {self.command.r_gfr}'
        currentCommand += f', rGWD = {self.command.r_gwd}'
        currentCommand += f', rCTR = {self.command.r_ctr}'

        self.get_logger().info(currentCommand)

        strAskForCommand = '-----\nAvailable commands\n\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0 - max width): Go to that position\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'
        strAskForCommand += '-->'

        return input(strAskForCommand)

    def gen_command(self, char, command):
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            self.get_logger().fatal("Select the gripper type from rg2 or rg6.")
            return command

        if char == 'c':
            command.r_gfr = max_force
            command.r_gwd = 0
            command.r_ctr = 16
        elif char == 'o':
            command.r_gfr = max_force
            command.r_gwd = max_width
            command.r_ctr = 16
        elif char == 'i':
            command.r_gfr += 25
            command.r_gfr = min(max_force, command.r_gfr)
            command.r_ctr = 16
        elif char == 'd':
            command.r_gfr -= 25
            command.r_gfr = max(0, command.r_gfr)
            command.r_ctr = 16
        else:
            try:
                command.r_gfr = max_force
                command.r_gwd = min(max_width, int(char))
                command.r_ctr = 16
            except ValueError:
                pass

        return command

    def publish_loop(self):
        try:
            char = self.ask_for_command()
            self.command = self.gen_command(char, self.command)
            self.pub.publish(self.command)
        except EOFError:
            self.get_logger().info("Shutting down...")
            raise KeyboardInterrupt


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGSimpleController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()