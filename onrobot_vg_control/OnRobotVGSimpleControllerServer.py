#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_control.msg import OnRobotVGOutput
from onrobot_vg_control.srv import SetCommand


class OnRobotVGNode(Node):
    def __init__(self):
        super().__init__('OnRobotVGSimpleControllerServer')
        
        self.pub = self.create_publisher(OnRobotVGOutput, 'OnRobotVGOutput', 10)
        self.command = OnRobotVGOutput()
        self.set_command_srv = self.create_service(
            SetCommand,
            "/onrobot_vg/set_command",
            self.handleSettingCommand)

    def handleSettingCommand(self, request, response):
        self.get_logger().info(f"Received VG command: {request.command}")
        self.command = self.genCommand(str(request.command), self.command)
        self.pub.publish(self.command)
        response.success = True
        response.message = f"VG command {request.command} executed successfully"
        return response

    def genCommand(self, char, command):
        # Commands work for both VG10 and VGC10
        # VG10 uses both channels, VGC10 typically uses channel A only
        
        if char == 'g':  # Grip all channels
            command.r_mca = 0x0100  # Grip mode
            command.r_vca = 255     # Max vacuum
            command.r_mcb = 0x0100  # Grip mode  
            command.r_vcb = 255     # Max vacuum
        elif char == 'r':  # Release all channels
            command.r_mca = 0x0000  # Release mode
            command.r_vca = 0
            command.r_mcb = 0x0000  # Release mode
            command.r_vcb = 0
        elif char == 'ga':  # Grip channel A only (for VGC10)
            command.r_mca = 0x0100
            command.r_vca = 255
        elif char == 'ra':  # Release channel A only
            command.r_mca = 0x0000
            command.r_vca = 0
        elif char == 'gb':  # Grip channel B only (for VG10 second cup)
            command.r_mcb = 0x0100
            command.r_vcb = 255
        elif char == 'rb':  # Release channel B only
            command.r_mcb = 0x0000
            command.r_vcb = 0
        elif char == 'ia':  # Idle channel A
            command.r_mca = 0x0200
        elif char == 'ib':  # Idle channel B
            command.r_mcb = 0x0200
        else:
            # If the command entered is a int, set vacuum level for all channels
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


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotVGNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()