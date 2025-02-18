#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from rclpy.parameter import ParameterType
import onrobot_rg_control.baseOnRobotRG
import rclpy.exceptions
import rclpy.parameter
import rclpy.time
from rclpy.action import GoalResponse, CancelResponse
from rclpy.callback_groups import ReentrantCallbackGroup

from std_srvs.srv import Trigger
from onrobot_rg_msgs.msg import OnRobotRGInput
from onrobot_rg_msgs.msg import OnRobotRGOutput
from sensor_msgs.msg import JointState
from control_msgs.action import GripperCommand
from onrobot_rg_msgs.msg import OnRobotRGOutput

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
        self.status : OnRobotRGInput
        
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
            self.dummy = self.declare_parameter('/onrobot/dummy', value=True).get_parameter_value().bool_value
        except:
            self.dummy = self.get_parameter('/onrobot/dummy').get_parameter_value().bool_value
            
        try:
            self.offset = self.declare_parameter('/onrobot/offset', value=50).get_parameter_value().integer_value
        except:
            self.offset = self.get_parameter('/onrobot/offset').get_parameter_value().integer_value
        
        self.gripper = onrobot_rg_control.baseOnRobotRG.onrobotbaseRG(self.gtype, self.dummy, self.offset)

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
        
        self.gripper_srv = rclpy.action.ActionServer(
            self, GripperCommand, '/onrobot_controller',
            callback_group=ReentrantCallbackGroup(),
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        
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
    
    def jointValueToWidth(self, joint_angle : float):
        return (np.cos(joint_angle + self.theta3) * self.L3 + self.dy + self.L1 * np.cos(self.theta1)) * 2
    
    
    def goal_callback(self, goal_request):
        self.get_logger().info('Received goal request')
        return GoalResponse.ACCEPT
    
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        command = OnRobotRGOutput()
        command.rctr = 8
        self.gripper.refreshCommand(command)
        self.gripper.sendCommand()
        self.prev_msg = self.gripper.message
        return CancelResponse.ACCEPT
    
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        goal = goal_handle.request.command
        feedback = GripperCommand.Feedback()
        result = GripperCommand.Result()
        
        command = OnRobotRGOutput()
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
        
        self.logger.info(str(int(self.jointValueToWidth(goal.position)*10000) - 2*self.offset))
            
        
        if 0 <= int(self.jointValueToWidth(goal.position)*10000) - 2*self.offset <= max_width:
            command.rgwd = int(self.jointValueToWidth(goal.position)*10000)
        elif self.widthToJointValue(0) > goal.position:
            command.rgwd = 0
        elif goal.position > self.widthToJointValue(max_width):
            command.rgwd = max_width
            
        command.rgfr = int(max_force/2)
        
        command.rctr = 16
        
        # if 0 <= goal.max_effort*10 <= max_force:
        #     command.rgfr = int(goal.max_effort*10)
        # elif 0 > goal.max_effort:
        #     command.rgfr = 0
        # elif 0 > goal.max_effort*10:
        #     command.rgfr = max_force
        
        self.gripper.refreshCommand(command)
        self.gripper.sendCommand()
        self.prev_msg = self.gripper.message
        
        self.logger.info(str(self.gripper.message))
        
        while rclpy.ok():
            
            if goal_handle.is_cancel_requested:
                # goal_handle.canceled()
                self.get_logger().error('Goal Canceled')
                return

            self.status = self.gripper.getStatus()
            self.pub.publish(self.status)
            self.publishJoints()
            
            if self.status is None:
                self.get_logger().warn("No gripper feedback yet")
                pass
            else:
                feedback.position = self.widthToJointValue(self.status.ggwd/10000)
                feedback.stalled = False
                self.logger.info(str(goal.position) + " " +str(feedback.position))
                # Position tolerance achieved or object grasped
                if (np.abs(goal.position - self.widthToJointValue(self.status.ggwd/10000)) < 0.01):
                    feedback.reached_goal = True
                    self.get_logger().debug('Goal achieved: %r'% feedback.reached_goal)

                goal_handle.publish_feedback(feedback)

                if feedback.reached_goal:
                    self.get_logger().debug('Reached goal, exiting loop')
                    break
    
                time.sleep(0.01)
        
        result.position = self.widthToJointValue(self.status.ggwd/10000)
        result.reached_goal = feedback.reached_goal
        result.stalled = feedback.stalled
        if result.reached_goal:
            self.get_logger().debug('Setting action to succeeded')
            goal_handle.succeed()
        else:
            self.get_logger().debug('Setting action to abort')
            goal_handle.abort()
        return result
    
    def publishJoints(self):
        msg = JointState()
        msg.header.stamp = self.time.to_msg()
        msg.name = self.joint_names
        if self.status.ggwd <= 1600:
            j_value = self.widthToJointValue((self.status.ggwd) / 10000)
            msg.position = [j_value * ratio for ratio in self.mimic_ratios]
            self.joint_states_pub.publish(msg)
    
    def mainLoop(self):
        """ Loops the sending status and command, and receiving message. """ 
            # Getting and publish the Gripper status
        self.status = self.gripper.getStatus()
        self.pub.publish(self.status)
        
        self.publishJoints()

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
