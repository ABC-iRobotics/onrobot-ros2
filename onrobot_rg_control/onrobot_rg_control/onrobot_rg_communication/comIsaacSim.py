#!/usr/bin/env python3

from onrobot_rg_control.onrobot_rg_communication.comBase import OnRobotCommunicationBase
from onrobot_rg_control.OnRobotRGSimpleControllerServer import OnRobotRGNode
from sensor_msgs.msg import JointState
from isaac_sim_msgs.srv import SetJoint

class communication(OnRobotCommunicationBase):
    
    def __init__(self, base : OnRobotRGNode):
        self.base = base
        self.busy = False
        self.state_sub = self.base.create_subscription(JointState, self.base.joint_states_sub, self.jointStateCallback, 3, callback_group=self.base.reentrant)
        self.joint_angle = 0.0
        self.last_joint_angle = 0.0
        self.joint_angle_desired = 0.0
        self.last_joint_angle_desired = 0.0
        self.jointState = JointState()
        self.lastJointState = JointState()
        self.rate = self.base.create_rate(20)
        self.counter = 0
        self.setJoint = self.base.create_client(SetJoint, '/IsaacSim/SetJoint', callback_group=self.base.reentrant)
        self.base.jointPub.destroy()
        
    def sendCommand(self, force : float, joint_angle : float, command_type):
        if command_type == 8:
            self.joint_angle_desired = self.joint_angle
        else:
            self.joint_angle_desired = joint_angle
            
        msg = JointState()
        msg.header.stamp = self.base.time.to_msg()
        msg.name = self.base.joint_names
        msg.position = [self.joint_angle_desired * ratio for ratio in self.base.mimic_ratios]
        msg.effort = [(float)(force) for num in self.base.mimic_ratios]
        
        srv = SetJoint.Request()
        srv.joint = "finger_joint"
        srv.value = joint_angle
        self.setJoint.call_async(srv)
        
        # self.base.joint_state_pub.publish(msg)
        
        
        
    def getStatus(self):
        return {
            'offset' : self.base.offset,
            'depth' : self.base.jointValueToHeight(self.joint_angle_desired)*1000,
            'relative_depth' : (self.base.jointValueToHeight(self.joint_angle_desired) - self.base.jointValueToHeight(self.last_joint_angle_desired))*1000,
            'relative_width' : self.base.jointValueToWidth(self.joint_angle_desired) - 2 * self.base.offset,
            'joint_angle' : (float)(self.joint_angle_desired),
            'busy' : self.busy,
            'grip' : False,
            's1_p' : False,
            's1_t' : False,
            's2_p' : False,
            's2_t' : False,
            'safety' : False,      
            'width' : self.base.jointValueToWidth(self.joint_angle_desired)
        }
        
    def restartPowerCycle(self):
        return True
    
    def jointStateCallback(self, message : JointState):
        for i in range(len(message.name)):
            if message.name[i] == 'finger_joint':
                self.last_joint_angle = self.joint_angle
                self.joint_angle = message.position[i]
                self.lastJointState = self.jointState
                self.jointState = message
                self.busy = False if (
                                (abs(self.joint_angle - self.last_joint_angle) < 1e-4 or 
                                abs(self.joint_angle_desired - self.joint_angle) < 1e-1) and 
                                abs(self.jointState.velocity[i]) < 5e-3
                                ) else True
        if self.counter % 20 == 1:
            self.base.get_logger().info('Gripper joints: ' + str([pos for pos in self.jointState.position if 'finger_joint' in self.jointState.name]) + '\nBusy: ' + str(self.busy))
            self.base.get_logger().info('Conditions:')
            self.base.get_logger().info('Position difference: ' + str(self.joint_angle - self.last_joint_angle))
            self.base.get_logger().info('Distance from goal: ' + str(self.joint_angle_desired - self.joint_angle))
            self.base.get_logger().info('Velocity: ' + str(self.jointState.velocity[0]))
            self.counter %= 20
        self.counter += 1