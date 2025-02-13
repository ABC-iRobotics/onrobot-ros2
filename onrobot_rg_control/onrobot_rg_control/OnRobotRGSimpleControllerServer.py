#!/usr/bin/env python3

from sympy import true
import rclpy
import time
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.srv import SetCommand
import rclpy.logging


class OnRobotRGNode(Node):
    """ OnRobotRGNode handles setting commands.

        Attributes:
            pub (rclpy.Publisher): the publisher for OnRobotRGOutput
            command (OnRobotRGOutput): command to be sent
            set_command_srv (rclpy.Service): set_command service instance

            handleSettingCommand:
                Handles sending commands via socket connection.
            genCommand:
                Updates the command according to the input character.
    """

    def __init__(self, nodeName : str):
        super().__init__(nodeName)
        try:
            self.gtype = self.declare_parameter('/onrobot/gripper', value='rg6').get_parameter_value().string_value
        except:
            self.gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value
            
        self.logger = rclpy.logging.get_logger("OnRobotRGServer")
        self.pub = self.create_publisher(OnRobotRGOutput, 'OnRobotRGOutput', 1)
        self.command = OnRobotRGOutput()
        self.set_command_srv = self.create_service(SetCommand, '/onrobot_rg/set_command', self.handleSettingCommand)


    def handleSettingCommand(self, req, res):
        """ Handles sending commands via socket connection. """

        self.logger.info(str(req.command))
        self.command = self.genCommand(str(req.command), self.command)
        self.pub.publish(self.command)
        time.sleep(1)
        res = SetCommand()
        res.Response.success = true
        res.Response.message = ""
        return res

    def genCommand(self, char, command):
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
            command.rgfr = max_force
            command.rgwd = 0
            command.rctr = 16
        elif char == 'o':
            command.rgfr = max_force
            command.rgwd = max_width
            command.rctr = 16
        elif char == 'i':
            command.rgfr += 25
            command.rgfr = min(max_force, command.rgfr)
            command.rctr = 16
        elif char == 'd':
            command.rgfr -= 25
            command.rgfr = max(0, command.rgfr)
            command.rctr = 16
        else:
            # If the command entered is a int, assign this value to rgwd
            try:
                command.rgfr = max_force
                command.rgwd = min(max_width, int(char))
                command.rctr = 16
            except ValueError:
                pass

        return command


def main():
    
    rclpy.init(args=None)
    node = OnRobotRGNode('OnRobotRGSimpleControllerServer')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
