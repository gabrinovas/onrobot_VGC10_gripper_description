#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from onrobot_vg_control.msg import OnRobotVGInput, OnRobotVGOutput
import onrobot_vg_control.baseOnRobotVG
import onrobot_vg_modbus_tcp.comModbusTcp


class OnRobotVGTcp(Node):
    def __init__(self):
        super().__init__('OnRobotVGTcpNode')
        
        # Get parameters
        self.declare_parameter('ip', '192.168.1.1')
        self.declare_parameter('port', '502')
        self.declare_parameter('changer_addr', '65')
        self.declare_parameter('dummy', False)
        
        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value
        changer_addr = self.get_parameter('changer_addr').value
        dummy = self.get_parameter('dummy').value

        # Gripper is a VG gripper with a Modbus/TCP connection
        self.gripper = onrobot_vg_control.baseOnRobotVG.onrobotbaseVG()
        self.gripper.client = onrobot_vg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotVGInput'
        self.pub = self.create_publisher(OnRobotVGInput, 'OnRobotVGInput', 10)

        # The Gripper command is received from the topic 'OnRobotVGOutput'
        self.subscription = self.create_subscription(
            OnRobotVGOutput,
            'OnRobotVGOutput',
            self.gripper.refreshCommand,
            10)

        self.timer = self.create_timer(0.05, self.mainLoop)
        self.prev_msg = []

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """
        # Get and publish the Gripper status
        status = self.gripper.getStatus()
        self.pub.publish(status)

        # Send the most recent command
        if not self.prev_msg == self.gripper.message:  # find new message
            self.get_logger().info("Sending message to VG gripper.")
            self.gripper.sendCommand()
        self.prev_msg = self.gripper.message


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotVGTcp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()