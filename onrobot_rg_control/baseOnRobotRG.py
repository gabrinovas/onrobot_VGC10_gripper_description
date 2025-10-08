#!/usr/bin/env python3

import rclpy
from onrobot_rg_control.msg import OnRobotRGInput, OnRobotRGOutput


class onrobotbaseRG:
    def __init__(self, gtype):
        self.gtype = gtype
        self.message = []
        self.client = None
        self.logger = rclpy.logging.get_logger('onrobotbaseRG')

    def verifyCommand(self, command):
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            self.logger.fatal("Select the gripper type from rg2 or rg6.")
            return command

        command.r_gfr = max(0, command.r_gfr)
        command.r_gfr = min(max_force, command.r_gfr)
        command.r_gwd = max(0, command.r_gwd)
        command.r_gwd = min(max_width, command.r_gwd)

        if command.r_ctr not in [1, 8, 16]:
            self.logger.fatal("Select the mode number from 1 (grip), 8 (stop), or 16 (grip_w_offset).")

        return command

    def refreshCommand(self, command):
        command = self.verifyCommand(command)
        self.message = []
        self.message.append(command.r_gfr)
        self.message.append(command.r_gwd)
        self.message.append(command.r_ctr)

    def sendCommand(self):
        if self.client:
            self.client.sendCommand(self.message)

    def restartPowerCycle(self):
        if self.client:
            self.client.restartPowerCycle()

    def getStatus(self):
        if not self.client:
            status = [0] * 18
        else:
            status = self.client.getStatus()

        message = OnRobotRGInput()
        if len(status) >= 18:
            message.g_fof = status[0]
            message.g_gwd = status[9]
            message.g_sta = status[10]
            message.g_wdf = status[17]

        return message