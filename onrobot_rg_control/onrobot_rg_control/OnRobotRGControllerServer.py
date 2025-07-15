#!/usr/bin/env python3

# ======================================================== #
# ==================== Python Imports ==================== #
# ======================================================== #
import numpy as np


# ======================================================== #
# ===================== ROS 2 Imports ==================== #
# ======================================================== #
import rclpy
import rclpy.logging
from rclpy.timer import Timer
from rclpy.node import Node
from rclpy.time import Time
from rclpy.executors import MultiThreadedExecutor, ExternalShutdownException
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import GoalResponse, CancelResponse

# ======================================================== #
# ==================== ROS 2 Messages ==================== #
# ======================================================== #
from std_srvs.srv import Trigger
from control_msgs.action import GripperCommand
from sensor_msgs.msg import JointState
from onrobot_rg_msgs.srv import GripperPose

from onrobot_rg_msgs.msg import OnRobotRGOutput
from onrobot_rg_msgs.srv import SetCommand

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
        
        self.reentrant = ReentrantCallbackGroup()
        
        self.initParams()
        
        self.time = Time()
        self.command = OnRobotRGOutput()
        self.command.rgfr = self.max_force
        
        self.logger = rclpy.logging.get_logger("OnRobotRGServer")
        
        # * Command from keyboard service
        self.set_command_srv = self.create_service(SetCommand, '/onrobot/sendCommand', self.sendCommandCallback, callback_group=self.reentrant)
        
        # * Restart gripper service
        self.restart_power_srv = self.create_service(Trigger, '/onrobot/restartPower', self.restartPowerCycle)
        
        # * GripperPose service
        self.gripper_pose = self.create_service(GripperPose, '/onrobot/pose', self.gripperPoseCallback)
        
        # * GripperCommand interface
        self.gripper_srv = rclpy.action.ActionServer(
            self, GripperCommand, '/rg6_controller',
            callback_group=self.reentrant,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            execute_callback=self.execute_callback)
        
        self.jointPub : Timer = self.create_timer(1.0/self.publish_freq, self.getStatus)

        from onrobot_rg_control.onrobot_rg_communication.comBase import OnRobotCommunicationBase
        
        self.gripper: OnRobotCommunicationBase
        
        match self.control:
            case 'modbus':
                self.initModbusTCP()
            case 'isaac':
                self.initIsaacSim()
                
        self.command.rgwd = (int)(min(self.max_width, self.gripper.getStatus()['width']))
        

    def __del__(self):
        match self.control:
            case 'modbus':
                self.gripper.disconnectFromDevice()

    def initParams(self):
        # * Common parameters
        try:
            self.gtype = self.declare_parameter('/onrobot/gripper', value='rg6').get_parameter_value().string_value
        except:
            self.gtype = self.get_parameter('/onrobot/gripper').get_parameter_value().string_value
            
        try:
            self.control = self.declare_parameter('/onrobot/control', value='tcp').get_parameter_value().string_value
        except:
            self.control = self.get_parameter('/onrobot/control').get_parameter_value().string_value
        
        try:
            self.offset = self.declare_parameter('/onrobot/offset', value=50).get_parameter_value().integer_value
        except:
            self.offset = self.get_parameter('/onrobot/offset').get_parameter_value().integer_value
            
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
            self.dz1 = 0.1095
            self.dz2 = 0.0427 

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
            
        self.publish_freq = 50 # Hz
        self.goalTolerance = 0.02 # Radians
        self.joint_names = [
                            "finger_joint", 
                            "left_inner_knuckle_joint", 
                            "left_inner_finger_joint", 
                            "right_outer_knuckle_joint", 
                            "right_inner_knuckle_joint", 
                            "right_inner_finger_joint"
                            ]
        self.mimic_ratios = [1, -1, 1, -1, -1, 1]
        
        self.joint_states_pub = self.create_publisher(JointState, '/joint_states', 3, callback_group=self.reentrant)
        
    def initModbusTCP(self):
        try:
            self.ip = self.declare_parameter('/onrobot/ip', value='192.168.1.1').get_parameter_value().string_value
        except:
            self.ip = self.get_parameter('/onrobot/ip').get_parameter_value().string_value
        
        try:
            self.port = self.declare_parameter('/onrobot/port', value=502).get_parameter_value().integer_value
        except:
            self.port = self.get_parameter('/onrobot/port').get_parameter_value().integer_value
        
        try:
            self.changer_addr = self.declare_parameter('/onrobot/changer_addr', value=65).get_parameter_value().integer_value
        except:
            self.changer_addr = self.get_parameter('/onrobot/changer_addr').get_parameter_value().integer_value
    
        from onrobot_rg_control.onrobot_rg_communication.comModbusTcp import communication
        self.gripper = communication(self)
        self.gripper.connectToDevice(self.ip, self.port, self.changer_addr)
        
    def initIsaacSim(self):
        try:
            self.isaac_joint_states_sub = self.declare_parameter('/onrobot/isaac_joint_states', value='/joint_states').get_parameter_value().string_value
        except:
            self.isaac_joint_states_sub = self.get_parameter('/onrobot/isaac_joint_states').get_parameter_value().string_value
        
        try:
            self.isaac_joint_states_pub = self.declare_parameter('/onrobot/isaac_joint_commands', value='/isaac_joint_commands').get_parameter_value().string_value
        except:
            self.isaac_joint_states_pub = self.get_parameter('/onrobot/isaac_joint_commands').get_parameter_value().string_value
        
        from onrobot_rg_control.onrobot_rg_communication.comIsaacSim import communication
        self.gripper = communication(self)
        
    def restartPowerCycle(self, req: Trigger.Request, res: Trigger.Response):
        self.gripper.restartPowerCycle()
        res.success = True
        res.message = ""
        return res
    
    def getStatus(self):
        self.status = self.gripper.getStatus()
        msg = JointState()
        msg.header.stamp = self.time.to_msg()
        msg.name = self.joint_names
        joint_angle = self.status['joint_angle']
        msg.position = [joint_angle * ratio for ratio in self.mimic_ratios]
        msg.effort = ([((float)(self.command.rgfr))/10 for _ in self.mimic_ratios]) if self.status['busy'] else ([0.0 for _ in self.mimic_ratios])
        self.joint_states_pub.publish(msg)
        
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
        
    def genCommand(self, char):

        """ Updates the command according to the input character.

            Args:
                char (str): set command service request message
                command (OnRobotRGOutput): command to be sent

            Returns:
                command: command message with parameters set
        """

        if char == 'c':
            self.command.rgfr = min(self.max_force, self.command.rgfr)
            self.command.rgwd = 0
            self.command.rctr = 16
        elif char == 'o':
            self.command.rgfr = min(self.max_force, self.command.rgfr)
            self.command.rgwd = self.max_width
            self.command.rctr = 16
        elif char == 'i':
            self.command.rgfr += 25
            self.command.rgfr = min(self.max_force, self.command.rgfr)
            self.command.rctr = 16
        elif char == 'd':
            self.command.rgfr -= 25
            self.command.rgfr = max(0, self.command.rgfr)
            self.command.rctr = 16
        else:
            # If the command entered is a int, assign this value to rgwd
            try:
                self.command.rgwd = min(self.max_width, int(char))
                self.command.rctr = 16
            except ValueError:
                pass

    def sendCommandCallback(self, req, res):
        """ Handles sending commands via socket connection. """
        
        self.genCommand(str(req.command))
        
        self.gripper.sendCommand(((float)(self.command.rgfr))/10, 
            self.widthToJointValue(((float)(self.command.rgwd))/10000), self.command.rctr)
        res.success = True
        res.message = ""
        return res

    """
        GripperCommand goal callback
    """
    def goal_callback(self, goal_request):
            self.get_logger().info('Received goal request')
            self.stop = False
            return GoalResponse.ACCEPT
    
    """
        GripperCommand cancel callback
    """
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Received cancel request')
        self.gripper.sendCommand(0, 0, 8)
        self.stop = True
        return CancelResponse.ACCEPT
    
    """
        GripperCommand movement execution
    """
    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal')
        freqency = self.create_rate(self.publish_freq)
        # --------- Inputs --------- #
        goal = goal_handle.request.command

        # --------- Outputs -------- #
        feedback = GripperCommand.Feedback()
        result = GripperCommand.Result()
        
        # Publish desired joint state
        self.gripper.joint_angle_desired = goal.position
        start_state = self.gripper.joint_angle
        
        self.get_logger().info('Sending movement')
        self.get_logger().info('Goal set to: ' + (str)(goal.position))
        
        self.gripper.sendCommand(self.max_force/20, goal.position, 16)
        
        while rclpy.ok():
            
            # Handling cancel requests
            if self.stop:
                goal_handle.canceled()
                self.get_logger().error('Goal Canceled')

                # Result message
                result.position = self.gripper.joint_angle
                result.reached_goal = False
                result.stalled = True
                
                return result
                
            if self.gripper.joint_angle is None:
                self.get_logger().warn("No gripper feedback yet")
                self.get_logger().info('No feedback')
                pass
            else:
            
                # Feedback message
                feedback.position = self.gripper.joint_angle
                feedback.stalled = False
                # Check if position tolerance achieved
                self.getStatus()
                if (not self.status['busy']) and ((abs(self.gripper.joint_angle - start_state) > 1e-2) or (abs(goal.position - start_state) < 1e-2)):
                    self.get_logger().info('Set reached_goal.\nBusy: ' + (str)(self.gripper.busy) + "\nJoint_angle: " + (str)(self.gripper.joint_angle) + "\nStart_state: " + (str)(start_state))
                    feedback.reached_goal = True
                    self.get_logger().debug('Goal achieved: %r'% feedback.reached_goal)
                goal_handle.publish_feedback(feedback)
                if feedback.reached_goal:
                    self.get_logger().debug('Reached goal, exiting loop')
                    break
            
            freqency.sleep()                         

        # Result message
        result.position = self.gripper.joint_angle
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
        

def main():
    try:
        rclpy.init(args=None)
        executor = MultiThreadedExecutor()
        node = OnRobotRGNode('OnRobotRGController')
        executor.add_node(node)
        executor.spin()
    
        node.destroy_node()
        rclpy.shutdown()
        
    except(KeyboardInterrupt, ExternalShutdownException):
        pass

if __name__ == '__main__':
    main()