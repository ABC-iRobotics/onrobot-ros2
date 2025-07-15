#!/usr/bin/env python3

import rclpy
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.srv import SetCommand
from rclpy.node import Node
import rclpy.logging
import time


class OnRobotRGController(Node):
    
    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        self.logger = rclpy.logging.get_logger("OnRobotRGController")
        try:
            self.gtype = self.declare_parameter('/onrobot/gripper', value='rg6').get_parameter_value().string_value
        except:
            self.gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value
        
        self.publisher = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.client = self.create_client(SetCommand, '/onrobot/sendCommand')
        self.command = OnRobotRGOutput()
        self.command.rgfr = 400
        self.command.rgwd = 1000
        
        self.sendCommand()

    def sendCommand(self):
        while True:
            #self.genCommand(self.askForCommand())
            #self.publisher.publish(self.command)
            command = SetCommand.Request()
            command.command = self.askForCommand()
            self.future = self.client.call_async(command)
            while not self.future.done:
                time.sleep(0.1)

    def genCommand(self, char):
        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotRGOutput): command to be sent

            Returns:
                command: command message with parameters set
        """
        max_force = 0
        max_width = 0
        if self.gtype == 'rg2':
            max_force = 400
            max_width = 1100
        elif self.gtype == 'rg6':
            max_force = 1200
            max_width = 1600
        else:
            self.logger.fatal(self.get_name() + ": Select the gripper type from rg2 or rg6.")
            rclpy.shutdown()

        if char == 'c':
            self.command.rgfr = 400
            self.command.rgwd = 0
            self.command.rctr = 16
        elif char == 'o':
            self.command.rgfr = 400
            self.command.rgwd = max_width
            self.command.rctr = 16
        elif char == 'i':
            self.command.rgfr += 25
            self.command.rgfr = min(max_force, self.command.rgfr)
            self.command.rctr = 16
        elif char == 'd':
            self.command.rgfr -= 25
            self.command.rgfr = max(0, self.command.rgfr)
            self.command.rctr = 16
        elif char == 's':
            self.command.rctr = 8
        else:
            # If the command entered is a int, assign this value to rgwd
            try:
                self.command.rgwd = min(max_width, int(char))
                self.command.rctr = 16
            except ValueError:
                pass


    def askForCommand(self):
        """ Asks the user for a command to send to the gripper.

            Args:
                command (OnRobotRGOutput): command to be sent

            Returns:
                input(strAskForCommand) (str): input command strings
        """

        currentCommand = 'Simple OnRobot RG Controller\n-----\nCurrent command:'
        currentCommand += ' rgfr = ' + str(self.command.rgfr)
        currentCommand += ', rgwd = ' + str(self.command.rgwd)
        currentCommand += ', rctr = ' + str(self.command.rctr)

        self.logger.info(currentCommand)

        strAskForCommand = '-----\nAvailable commands\n\n'
        strAskForCommand += 'c: Close\n'
        strAskForCommand += 'o: Open\n'
        strAskForCommand += '(0 - max width): Go to that position\n'
        strAskForCommand += 'i: Increase force\n'
        strAskForCommand += 'd: Decrease force\n'
        strAskForCommand += 's: Stop movement\n'

        strAskForCommand += '-->'

        return input(strAskForCommand)  


def main():
    
    rclpy.init(args=None)
    node = OnRobotRGController('OnRobotRGController')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
