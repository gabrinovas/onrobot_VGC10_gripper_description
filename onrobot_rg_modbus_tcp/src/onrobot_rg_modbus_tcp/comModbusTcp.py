#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import threading
from pymodbus.client import ModbusTcpClient
import rclpy


class communication:
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection
            dummy (book): the process will be dummy mode (True) or not
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
            restartPowerCycle: Restarts the power cycle of Compute Box.
            getStatus: Sends a request to read and returns the gripper status.
    """

    def __init__(self, dummy=False):
        self.client = None
        self.dummy = dummy
        self.lock = threading.Lock()
        self.logger = rclpy.logging.get_logger('comModbusTcp')

    def connectToDevice(self, ip, port, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (int): port number (e.g. 502)
                changer_addr (int): quick tool changer address
        """

        if self.dummy:
            self.logger.info("Dummy mode: connectToDevice")
            return

        self.client = ModbusTcpClient(
            host=ip,
            port=port,
            timeout=1)
        self.changer_addr = changer_addr
        self.client.connect()
        if self.client.connected:
            self.logger.info(f"Connected to {ip}:{port} with changer address {changer_addr}")
        else:
            self.logger.error(f"Failed to connect to {ip}:{port}")

    def disconnectFromDevice(self):
        """ Closes connection. """

        if self.dummy:
            self.logger.info("Dummy mode: disconnectFromDevice")
            return

        if self.client:
            self.client.close()
            self.logger.info("Disconnected from device")

    def sendCommand(self, message):
        """ Sends a command to the Gripper.

            Args:
                message (list[int]): message to be sent
        """

        if self.dummy:
            self.logger.info("Dummy mode: sendCommand")
            return

        # Sending a command to the device (address 0 ~ 2)
        if message != [] and self.client and self.client.connected:
            try:
                with self.lock:
                    result = self.client.write_registers(
                        address=0, values=message, slave=self.changer_addr)
                    if result.isError():
                        self.logger.error(f"Modbus write error: {result}")
            except Exception as e:
                self.logger.error(f"Error sending command: {e}")
            self.logger.debug(f"Sent command: {message}")

    def restartPowerCycle(self):
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        message = 2
        restart_address = 63

        if self.dummy:
            self.logger.info("Dummy mode: restartPowerCycle")
            return

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        if self.client and self.client.connected:
            with self.lock:
                result = self.client.write_registers(
                    address=0, values=[message], slave=restart_address)
                if not result.isError():
                    self.logger.info("Restarted power cycle")
                else:
                    self.logger.error(f"Failed to restart power cycle: {result}")

    def getStatus(self):
        """ Sends a request to read and returns the gripper status. """

        response = [0] * 18
        if self.dummy:
            self.logger.info("Dummy mode: getStatus")
            return response

        if not self.client or not self.client.connected:
            self.logger.warning("Not connected to device")
            return response

        # Getting status from the device (address 258 ~ 275)
        try:
            with self.lock:
                result = self.client.read_holding_registers(
                    address=258, count=18, slave=self.changer_addr)
                if not result.isError():
                    response = result.registers
                else:
                    self.logger.error(f"Modbus error: {result}")
        except Exception as e:
            self.logger.error(f"Error reading status: {e}")

        return response