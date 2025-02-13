#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import ParameterType
import onrobot_rg_control.baseOnRobotRG
import rclpy.exceptions
import rclpy.parameter
import rclpy.time
from std_srvs.srv import Trigger
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput
from sensor_msgs.msg import JointState
import time
import numpy as np


class OnRobotRGTcp(Node):
    """ OnRobotRGTcp connects to the gripper with Modbus/TCP.

        Attributes:
            gripper (onrobot_rg_control.baseOnRobotRG.onrobotbaseRG):
                instance of onrobotbaseRG used for the connection establishment
            pub (rclpy.Publisher): the publisher for OnRobotRGInput

            restartPowerCycle:
                Restarts the power cycle of the gripper.
            mainLoop:
                Loops the sending status and command, and receiving message.
    """

    def __init__(self, nodeName : str):
        # Gripper is a RG gripper with a Modbus/TCP connection
        super().__init__(nodeName)
        
        self.logger = self.get_logger()
        self.time = Time()
        
        try:
            self.ip = self.declare_parameter('/onrobot/ip', value='192.168.1.1').get_parameter_value().string_value
        except:
            self.ip = self.get_parameter('/onrobot/ip').get_parameter_value().string_value
        
        try:
            self.port = self.declare_parameter('/onrobot/port', value=502).get_parameter_value().integer_value
        except:
            self.port = self.get_parameter('/onrobot/port').get_parameter_value().integer_value
        
        try:
            self.gtype = self.declare_parameter('/onrobot/gripper', value='rg6').get_parameter_value().string_value
        except:
            self.gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value

        try:
            self.changer_addr = self.declare_parameter('/onrobot/changer_addr', value=65).get_parameter_value().integer_value
        except:
            self.changer_addr = self.get_parameter('/onrobot/changer_addr').get_parameter_value().integer_value

        try:
            self.dummy = self.declare_parameter('/onrobot/dummy', value=False).get_parameter_value().bool_value
        except:
            self.dummy = self.get_parameter('/onrobot/dummy').get_parameter_value().bool_value
            
        try:
            self.dummy = self.declare_parameter('/onrobot/offset', value=5).get_parameter_value().integer_value
        except:
            self.dummy = self.get_parameter('/onrobot/offset').get_parameter_value().integer_value
        
        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(self.gtype, self.dummy)

        if not self.dummy:
            # Connecting to the ip address received as an argument
            self.logger.info("Connecting to: " + self.ip + ":" + str(self.port))
            self.gripper.client.connectToDevice(self.ip, self.port)

        # The Gripper status is published on the topic 'OnRobotRGInput'
        self.pub = self.create_publisher(OnRobotRGInput, 'OnRobotRGInput', 1)

        # The Gripper command is received from the topic 'OnRobotRGOutput'
        self.sub = self.create_subscription(OnRobotRGOutput, 'OnRobotRGOutput', self.gripper.refreshCommand, 1)
        
        self.joint_states_pub = self.create_publisher(JointState, 'joint_states', 5)

        # The restarting service
        self.srv = self.create_service(Trigger, '/onrobot_rg/restart_power', self.restartPowerCycle)
        
        self.joint_names = ["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint", 
                            "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]
        self.mimic_ratios = [1, -1, 1, -1, -1, 1]
        
        if self.gtype == 'rg6':
            self.L1 = 0.1394215     # Length between knuckle joint - drive joint (finger_joint) lateral intersect and upper plate middlepoint
            self.L3 = 0.08          # Length of knuckle link, joint-to-joint
            self.theta1 = 1.3963    # Angle between upper plate middlepoint, knuckle joint - drive joint (finger_joint) lateral intersect and middle axis
            self.theta3 = 0.93766   # Angle of knuckle in default pose
            self.dy = -0.0196       # Distande between the finger's inner side and the knuckle joint
        elif self.gtype == 'rg2':
            self.L1 = 0.108505
            self.L3 = 0.055
            self.theta1 = 1.41371
            self.theta3 = 0.76794
            self.dy = -0.0144

        self.prev_msg = []
        self.loop = self.create_timer(0.05, self.mainLoop)

    def restartPowerCycle(self, request, response):
        """ Restarts the power cycle of the gripper. """

        self.logger.info("Restarting the power cycle of all grippers connected.")
        self.gripper.restartPowerCycle()
        time.sleep(1)
        response = Trigger()
        response.Response.success = True
        response.Response.message = ""
        return response

    def widthToJointValue(self, width):
        return np.arccos(((width/2) - self.dy - self.L1 * np.cos(self.theta1)) / self.L3) - self.theta3
    
    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """ 
            # Getting and publish the Gripper status
        status = self.gripper.getStatus()
        self.pub.publish(status)
        
        msg = JointState()
        msg.header.stamp = self.time.to_msg()
        msg.name = self.joint_names
        if status.gwdf < 1600:
                j_value = self.widthToJointValue(status.gwdf / 10000)
                msg.position = [j_value * ratio for ratio in self.mimic_ratios]
                self.joint_states_pub.publish(msg)

        #time.sleep(0.05)
        # Sending the most recent command
        #if not int(format(status.gsta, '016b')[-1]):  # not busy
        if not self.prev_msg == self.gripper.message:  # find new message
            self.logger.info(self.get_name()+": Sending message.")
            self.gripper.sendCommand()
        self.prev_msg = self.gripper.message
        #time.sleep(0.05)


def main():
    rclpy.init(args=None)
    node = OnRobotRGTcp('OnRobotRGTcp')
    rclpy.spin(node)
    
    node.destroy_node()
    rclpy.shutdown()
