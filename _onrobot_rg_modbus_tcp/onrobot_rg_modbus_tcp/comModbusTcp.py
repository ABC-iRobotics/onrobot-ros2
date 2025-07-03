#!/usr/bin/env python3
"""
Module comModbusTcp: defines a class which communicates with
OnRobot Grippers using the Modbus/TCP protocol.
"""

import sys
from rclpy import logging
import threading
from pymodbus.client import ModbusTcpClient


class communication:
    """ communication sends commands and receives the status of RG gripper.

        Attributes:
            client (pymodbus.client.sync.ModbusTcpClient):
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

    def __init__(self, dummy):
        self.client : ModbusTcpClient
        self.dummy = dummy
        self.lock = threading.Lock()
        self.logger = logging.get_logger("OnRobot Modbus")

    def connectToDevice(self, ip, port : int, changer_addr=65):
        """ Connects to the client device (gripper).

            Args:
                ip (str): IP address (e.g. '192.168.1.1')
                port (str): port number (e.g. '502')
                changer_addr (int): quick tool changer address
        """

        if self.dummy:
            self.logger.info(
                self.logger.name +
                ": " +
                sys._getframe().f_code.co_name
                + ": Using Fake Gripper.")
            return
        self.changer_addr = changer_addr
        self.logger.info("Changer address: " + str(self.changer_addr))
        self.client = ModbusTcpClient(
            host=ip,
            port=port,
            timeout=1,
            #source_address=('gripper', self.changer_addr)
        )
        self.client.connect()
        if self.client.connected:
            self.logger.info("Connected successfully.")
        else:
            self.logger.info("Connection failed!")

    def disconnectFromDevice(self):
        """ Closes connection. """

        if self.dummy:
            self.logger.info(
                self.logger.name +
                ": " +
                sys._getframe().f_code.co_name
                + ": Closing Fake Gripper"
                )
            return

        self.client.close()

    def sendCommand(self, message):
        """ Sends a command to the Gripper.

            Args:
                message (list[int]): message to be sent
        """

        if self.dummy:
            self.logger.info(
                self.logger.name +
                ": " +
                sys._getframe().f_code.co_name
                + ": Sending Command"
                )
            return
        self.logger.info("Send Command")
        # Sending a command to the device (address 0 ~ 2)
        if message != []:
            with self.lock:
                self.client.write_registers(address=0, values=message, slave=self.changer_addr)

    def restartPowerCycle(self):
        """ Restarts the power cycle of Compute Box.

            Necessary is Safety Switch of the grippers are pressed
            Writing 2 to this field powers the tool off
            for a short amount of time and then powers them back
        """

        message = [2]
        restart_address = 63

        # Sending 2 to address 0x0 resets compute box (address 63) power cycle
        with self.lock:
            self.client.write_register(address=0, values=message, slave=self.changer_addr)

    def getStatus(self):
        """ Sends a request to read and returns the gripper status. """
        response = [0] * 18
        if self.dummy:
            self.logger.info(
                self.logger.name +
                ": " +
                sys._getframe().f_code.co_name
                + ": Getting Status"
                )
            return response

        # Getting status from the device (address 258 ~ 275)
        with self.lock:
            response = self.client.read_holding_registers(
                address=258, count=18, slave=self.changer_addr)
        # Output the result
        return response
