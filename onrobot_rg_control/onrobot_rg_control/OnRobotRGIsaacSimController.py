#!/usr/bin/env python3

# ======================================================== #
# =====================  ROS2 imports ==================== #
# ======================================================== #

import rclpy
import rclpy.clock
from rclpy.time import Time
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import GoalResponse, CancelResponse

# ======================================================== #
# ====================  Message types ==================== #
# ======================================================== #

from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from onrobot_rg_msgs.srv import GripperPose

# ======================================================== #
# ==================== General imports =================== #
# ======================================================== #

import numpy as np
from time import sleep


"""
    This ROS2 node interfaces Isaac Sim and MoveIt2 by implementing the neccessary GripperCommand 
"""
class FakeGripper(Node):
        
        def __init__(self, nodename : str) -> None:
            
            super().__init__(nodename)


            # * Get gripper type from ROS2 parameters
            try:
                self.gtype = self.declare_parameter('/onrobot/gripper', value='rg6').get_parameter_value().string_value
            except:
                self.gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value

            self.logger = self.get_logger()
            self.time = Time()
            self.jointState : JointState = JointState()
            
            self.reentrant_group = ReentrantCallbackGroup()


            # * GripperCommand interface
            self.gripper_srv = rclpy.action.ActionServer(
                self, GripperCommand, '/onrobot_controller',
                callback_group=self.reentrant_group,
                goal_callback=self.goal_callback,
                cancel_callback=self.cancel_callback,
                execute_callback=self.execute_callback)
                
            # * GripperPose service
            self.gripper_pose = self.create_service(GripperPose, '/onrobot/pose', self.gripperPoseCallback)
            
            # * Joint commands publisher for Isaac Sim control
            self.joint_states_pub = self.create_publisher(JointState, 'joint_command', 10, callback_group=self.reentrant_group)
            # * Joint states subscriber for Isaac Sim feedback
            self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self.getJoints, 10, callback_group=self.reentrant_group)
            
            

            # * Setting up gripper parameters
            self.max_force : int
            self.max_width : int
            
            if self.gtype == 'rg6':
                self.L1 = 0.1394215
                self.L3 = 0.080
                self.theta1 = 1.3963
                self.theta3 = 0.93766
                self.dy = -0.0196
                self.dz1 = 0.121522
                self.dz2 = 0.0539

                self.max_force = 1200
                self.max_width = 1600
                
                '''
                    Linear regression data from RG6 datasheet using points (force, speed@max_width):
                    0,      0
                    25,     30
                    80,     90
                    120,    135
                '''
                self.force_scaling = 0.1127
            
            elif self.gtype == 'rg2':
                self.L1 = 0.108505
                self.L3 = 0.055
                self.theta1 = 1.41371
                self.theta3 = 0.76794
                self.dy = -0.0144

                self.max_force = 400
                self.max_width = 1100
                
                '''
                    Linear regression data from RG6 datasheet using points (force, speed@max_width):
                    0,      0
                    3,     12.5
                    20,     66.5
                    40,    129.5
                '''
                self.force_scaling = 0.3259
                
            
            # Constants used to define behavior
            self.publish_freq = 50 # Hz
            self.goalTolerance = 0.02 # Radians
            self.joint_names = ["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint", "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]
            self.mimic_ratios = [1, -1, 1, -1, -1, 1]

            # Init control variables
            self.joint_angle = 0
            self.stop = False
        

        """
            Convert gripper width to joint value
            @param width Gripper width in m
        """
        def widthToJointValue(self, width : float) -> float:
            return np.arccos(((width/2) - self.dy - self.L1 * np.cos(self.theta1)) / self.L3) - self.theta3
        
        """
            Convert joint value to gripper width
            @param joint_angle Joint value in radians
        """
        def jointValueToWidth(self, joint_angle : float) -> float:
            return (np.cos(joint_angle + self.theta3) * self.L3 + self.dy + self.L1 * np.cos(self.theta1)) * 2

        """
            Convert gripper height to joint value
            @param height Gripper height in m
        """
        def heightToJointValue(self, height : float) -> float:
            return np.arcsin((height - (self.dz1 + self.dz2))/self.L3) - self.theta3
        
        """
            Convert joint value to gripper height
            @param joint_angle Joint value in radians
        """
        def jointValueToHeight(self, joint_angle : float) -> float:
            return self.L3 * np.sin(joint_angle + self.theta3) + (self.dz1 + self.dz2)
            
        """
            GripperCommand goal callback
        """
        def goal_callback(self, goal_request):
                self.get_logger().info('Received goal request')
                return GoalResponse.ACCEPT
        
        """
            GripperCommand cancel callback
        """
        def cancel_callback(self, goal_handle):
            self.get_logger().info('Received cancel request')
            self.stop = True
            return CancelResponse.ACCEPT
        
        """
            GripperCommand movement execution
        """
        def execute_callback(self, goal_handle):
            self.get_logger().info('Executing goal')

            # --------- Inputs --------- #
            goal = goal_handle.request.command

            # --------- Outputs -------- #
            feedback = GripperCommand.Feedback()
            result = GripperCommand.Result()
            
            # Publish desired joint state
            self.joint_angle_desired = goal.position
            self.publishJoints()
            start_state = self.joint_angle
            self.stopped = False
            
            while rclpy.ok():
                
                # Handling cancel requests
                if self.stop:
                    self.stop = False                               # Clear request
                    goal_handle.canceled()
                    self.get_logger().error('Goal Canceled')

                    self.joint_angle_desired = self.joint_angle     # Set desired to current joint state
                    self.publishJoints()                            # Stop Isaac Sim movement by setting target position to current
                    
                    # Result message
                    result.position = self.joint_angle
                    result.reached_goal = False
                    result.stalled = True
                    
                    return result

                if self.joint_angle is None:
                    self.get_logger().warn("No gripper feedback yet")
                    pass
                else:
                
                    # Feedback message
                    feedback.position = self.joint_angle
                    feedback.stalled = False
                   
                    # Check if position tolerance achieved
                    if self.stopped and not self.joint_angle == start_state:
                        feedback.reached_goal = True
                        self.get_logger().debug('Goal achieved: %r'% feedback.reached_goal)
                    goal_handle.publish_feedback(feedback)

                    if feedback.reached_goal:
                        self.get_logger().debug('Reached goal, exiting loop')
                        break
                    
                    self.publishJoints()                            # Sometimes Isaac Sim don't recive first publish
                
                
                sleep(1/self.publish_freq)                          # Wait until next feedback - default: 2 ms

            # Result message
            result.position = self.joint_angle
            result.reached_goal = feedback.reached_goal
            result.stalled = feedback.stalled
            if result.reached_goal:
                self.get_logger().debug('Setting action to succeeded')
                goal_handle.succeed()
            else:
                self.get_logger().debug('Setting action to abort')
                goal_handle.abort()
            return result
        
        """
            Requests one parameter from: width, height, joint angle. If the first 2 is zero, then joint_angle = 0 is assumed. Returns the full pose of the gripper TCP.
        """
        def gripperPoseCallback(self, request : GripperPose.Request, response : GripperPose.Response):
            if request.known.x < 0 or request.known.y < 0:
                self.get_logger().error('Negative value were given!')
                return response
            if abs(request.known.theta) > 0.628319:
                self.get_logger().error('Joint angle out of bound')
                return response
            if (not request.known.x == 0) and ((not request.known.y == 0) or (not request.known.theta == 0)) or ((not request.known.y == 0) and (not request.known.theta == 0)):
                self.get_logger().error('More than one parameter given! Please define either width (x), height (y), or joint angle (theta).')
                return response
            if not request.known.x == 0:
                response.pose.x = request.known.x
                response.pose.theta = self.widthToJointValue(request.known.x)
                response.pose.y = self.jointValueToHeight(response.pose.theta)
                return response
            if not request.known.y == 0:
                response.pose.y = request.known.y
                response.pose.theta = self.heightToJointValue(request.known.y)
                response.pose.x = self.jointValueToWidth(response.pose.theta)
                return response
            response.pose.theta = request.known.theta
            response.pose.x = self.jointValueToWidth(response.pose.theta)
            response.pose.y = self.jointValueToHeight(response.pose.theta)
            return response
        
        """
            Publish joint states to "joint_command" topic
            This will update Isaac Sim's target position for the RG gripper
        """
        def publishJoints(self) -> None:
            msg = JointState()
            msg.header.stamp = self.time.to_msg()
            msg.name = self.joint_names

            j_value = self.joint_angle_desired
            msg.position = [j_value * ratio for ratio in self.mimic_ratios]

            self.joint_states_pub.publish(msg)

        """
            Get joint states from "joint_states" topic
            This is the main feedback from Isaac Sim
        """
        def getJoints(self, message : JointState) -> None:
            self.lastJointState = self.jointState
            self.jointState = message
            for i in range(len(message.name)):
                if message.name[i] == 'finger_joint':
                    self.joint_angle = message.position[i]
            self.stopped = True if all(abs(pos - last_pos) < 1e-2 and 
                                    abs(vel - last_vel) < 1e-2 for pos, last_pos, vel, last_vel, joint in 
                                    zip(self.jointState.position, self.lastJointState.position, self.jointState.velocity, self.lastJointState.velocity, self.jointState.name) 
                                    if joint == 'finger_joint') else False

def main():
    #Init ROS2
    rclpy.init(args=None)
    
    #Using MultiThreadedExecutor
    executor = MultiThreadedExecutor()
    node = FakeGripper('OnRobotRGFakeGripper')
    executor.add_node(node)
    executor.spin()
    
    #Exiting
    node.destroy_node()
    rclpy.shutdown()