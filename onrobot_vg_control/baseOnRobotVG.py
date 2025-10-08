#!/usr/bin/env python3

import rclpy
from onrobot_vg_control.msg import OnRobotVGInput, OnRobotVGOutput


class onrobotbaseVG:
    def __init__(self):
        self.message = []
        self.client = None
        self.logger = rclpy.logging.get_logger('onrobotbaseVG')

    def verifyCommand(self, command):
        # Verifying that each variable is in its correct range
        command.r_vca = max(0, command.r_vca)
        command.r_vca = min(255, command.r_vca)
        command.r_vcb = max(0, command.r_vcb)
        command.r_vcb = min(255, command.r_vcb)

        # Verifying that the selected mode number is available
        valid_modes = [0x0000, 0x0100, 0x0200]  # Release, Grip, Idle
        
        if command.r_mca not in valid_modes:
            self.logger.fatal(
                "Select the mode number for ch A from: "
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")
            
        if command.r_mcb not in valid_modes:
            self.logger.fatal(
                "Select the mode number for ch B from: "
                "0x0000 (release), 0x0100 (grip), or 0x0200 (idle).")

        return command

    def refreshCommand(self, command):
        command = self.verifyCommand(command)
        self.message = []
        self.message.append(command.r_mca)
        self.message.append(command.r_vca)
        self.message.append(command.r_mcb)
        self.message.append(command.r_vcb)

    def sendCommand(self):
        if self.client:
            self.client.sendCommand(self.message)

    def getStatus(self):
        if not self.client:
            status = [0] * 2
        else:
            status = self.client.getStatus()

        message = OnRobotVGInput()
        if len(status) >= 2:
            message.g_vca = status[0]
            message.g_vcb = status[1]

        return message