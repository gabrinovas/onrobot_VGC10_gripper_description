#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_rg_control.msg import OnRobotRGOutput
from onrobot_rg_control.srv import SetCommand


class OnRobotRGNode(Node):
    def __init__(self):
        super().__init__('OnRobotRGSimpleControllerServer')
        
        self.declare_parameter('gripper', 'rg6')
        self.gtype = self.get_parameter('gripper').value
        
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 10)
        self.command = OnRobotRGOutput()
        self.set_command_srv = self.create_service(
            SetCommand,
            "/onrobot_rg/set_command",
            self.handleSettingCommand)

    def handleSettingCommand(self, request, response):
        self.get_logger().info(f"Received command: {request.command}")
        self.command = self.genCommand(str(request.command), self.command)
        self.pub.publish(self.command)
        response.success = True
        response.message = f"Command {request.command} executed successfully"
        return response

    def genCommand(self, char, command):
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
            # If the command entered is a int, assign this value to rGWD
            try:
                command.r_gfr = max_force
                command.r_gwd = min(max_width, int(char))
                command.r_ctr = 16
            except ValueError:
                pass

        return command


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()