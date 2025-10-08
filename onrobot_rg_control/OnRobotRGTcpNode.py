#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from onrobot_rg_control.msg import OnRobotRGInput, OnRobotRGOutput
import onrobot_rg_control.baseOnRobotRG
import onrobot_rg_modbus_tcp.comModbusTcp


class OnRobotRGTcp(Node):
    def __init__(self):
        super().__init__('OnRobotRGTcpNode')
        
        # Get parameters
        self.declare_parameter('ip', '192.168.1.1')
        self.declare_parameter('port', '502')
        self.declare_parameter('gripper', 'rg6')
        self.declare_parameter('changer_addr', '65')
        self.declare_parameter('dummy', False)
        
        ip = self.get_parameter('ip').value
        port = self.get_parameter('port').value
        gtype = self.get_parameter('gripper').value
        changer_addr = self.get_parameter('changer_addr').value
        dummy = self.get_parameter('dummy').value

        # Gripper is a RG gripper with a Modbus/TCP connection
        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(gtype)
        self.gripper.client = onrobot_rg_modbus_tcp.comModbusTcp.communication(dummy)

        # Connecting to the ip address
        self.gripper.client.connectToDevice(ip, port, changer_addr)

        # The Gripper status is published on the topic 'OnRobotRGInput'
        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 10)

        # The Gripper command is received from the topic 'OnRobotRGOutput'
        self.subscription = self.create_subscription(
            OnRobotRGOutput,
            'OnRobotRGOutput',
            self.gripper.refreshCommand,
            10)

        # The restarting service
        self.srv = self.create_service(
            Trigger,
            "/onrobot_rg/restart_power",
            self.restartPowerCycle)

        self.timer = self.create_timer(0.05, self.mainLoop)
        self.prev_msg = []

    def restartPowerCycle(self, request, response):
        """ Restarts the power cycle of the gripper. """
        self.get_logger().info("Restarting the power cycle of the gripper.")
        self.gripper.restartPowerCycle()
        response.success = True
        response.message = "Power cycle restarted"
        return response

    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """
        # Getting and publish the Gripper status
        status = self.gripper.getStatus()
        self.pub.publish(status)

        # Sending the most recent command
        if not int(format(status.g_sta, '016b')[-1]):  # not busy
            if not self.prev_msg == self.gripper.message:  # find new message
                self.get_logger().info("Sending message to gripper.")
                self.gripper.sendCommand()
        self.prev_msg = self.gripper.message


def main(args=None):
    rclpy.init(args=args)
    node = OnRobotRGTcp()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()