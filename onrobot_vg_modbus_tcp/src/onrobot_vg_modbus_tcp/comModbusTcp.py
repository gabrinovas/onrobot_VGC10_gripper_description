#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot VG grippers using the Modbus/TCP protocol.
"""

import threading
from pymodbus.client.sync import ModbusTcpClient
import rclpy


class communication:
    """ communication sends commands and receives the status of VG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
                instance of ModbusTcpClient to establish modbus connection
            dummy (book): the process will be dummy mode (True) or not
            lock (threading.Lock):
                instance of the threading.Lock to achieve exclusive control

            connectToDevice: Connects to the client device (gripper).
            disconnectFromDevice: Closes connection.
            sendCommand: Sends a command to the Gripper.
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
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
        """

        if self.dummy:
            self.logger.info("Dummy mode: connectToDevice")
            return

        self.client = ModbusTcpClient(
            ip,
            port=port,
            stopbits=1,
            bytesize=8,
            parity='E',
            baudrate=115200,
            timeout=1)
        self.changer_addr = changer_addr
        self.client.connect()
        self.logger.info(f"Connected to VG gripper at {ip}:{port} with changer address {changer_addr}")

    def disconnectFromDevice(self):
        """ Closes connection. """

        if self.dummy:
            self.logger.info("Dummy mode: disconnectFromDevice")
            return

        if self.client:
            self.client.close()
            self.logger.info("Disconnected from VG gripper")

    def sendCommand(self, message):
        """ Sends a command to the VG Gripper.

            Args:
                message (list[int]): message to be sent [rMCA, rVCA, rMCB, rVCB]
        """

        if self.dummy:
            self.logger.info(f"Dummy mode: sendCommand - would send {message}")
            return

        # Sending a command to the device (address 0 ~ 1)
        # VG gripper uses combined registers: [rMCA + rVCA, rMCB + rVCB]
        if message != [] and self.client and len(message) == 4:
            try:
                # Combine mode and vacuum values for each channel
                command_a = message[0] + message[1]  # rMCA + rVCA
                command_b = message[2] + message[3]  # rMCB + rVCB
                
                command = [command_a, command_b]
                
                with self.lock:
                    self.client.write_registers(
                        address=0, values=command, unit=self.changer_addr)
                self.logger.debug(f"Sent VG command: {command} (original: {message})")
            except Exception as e:
                self.logger.error(f"Error sending VG command: {e}")
        else:
            self.logger.warn(f"Invalid VG command message: {message}")

    def getStatus(self):
        """ Sends a request to read and returns the VG gripper status.

            Returns:
                list[int]: [gVCA, gVCB] - vacuum values for channels A and B
        """

        response = [0] * 2
        if self.dummy:
            self.logger.info("Dummy mode: getStatus - returning dummy values")
            # Return realistic dummy values for testing
            return [850, 0]  # Channel A has good vacuum, Channel B unused

        if not self.client:
            self.logger.warning("Not connected to VG gripper")
            return response

        # Getting status from the device (address 258 ~ 259)
        try:
            with self.lock:
                result = self.client.read_holding_registers(
                    address=258, count=2, unit=self.changer_addr)
                if not result.isError():
                    response = result.registers
                    self.logger.debug(f"Received VG status: {response}")
                else:
                    self.logger.error(f"Modbus error reading VG status: {result}")
        except Exception as e:
            self.logger.error(f"Error reading VG status: {e}")

        return response