#!/usr/bin/env python3

import rclpy
import numpy as np
from itertools import chain
from rclpy.node import Node
from onrobot_rg_msgs.msg import OnRobotRGInput, OnRobotRGOutput
from onrobot_rg_control.onrobot_rg_communication import comModbusTcp
import rclpy.logging
from bisect import bisect_left
from multiprocessing import Process, Value, Manager
import time
from ctypes import c_bool


class onrobotbaseRG(Node):
    """ onrobotbaseRG sends commands and receives the status of RG gripper.

        Attributes:
            gtype (str): gripper type 'rg2' or 'rg6'
            message (list[int]): message including commands to be sent

            verifyCommand:
                Verifies that the value of each variable satisfy its limits.
            refreshCommand:
                Updates the command sent during the next sendCommand() call.
    """

    def __init__(self, gtype : str, dummy : bool, offset = 50):
        
        super().__init__('onrobotbaseRG')
        self.logger = self.get_logger()
        # Initiating output message as an empty list
        self.gtype = gtype
        self.dummy = dummy
        self.offset = offset
        
        self.max_force = 0
        self.max_width = 0
        # Verifying that each variable is in its correct range
        if self.gtype == 'rg2':
            self.max_force = 400
            self.max_width = 1100
        elif self.gtype == 'rg6':
            self.max_force = 1200
            self.max_width = 1600
        else:
            self.logger.fatal(self.get_name() + ": Select the gripper type from rg2 or rg6.")
            rclpy.shutdown()
        
        self.message = []
        self.client = comModbusTcp.communication()
        
    def verifyCommand(self, command : OnRobotRGOutput) -> OnRobotRGOutput:
        """ Verifies that the value of each variable satisfy its limits.

            Args:
                command (OnRobotRGOutput): command message to be verified

            Returns:
                command (OnRobotRGOutput): verified command message
        """
        command.rgfr = max(0, command.rgfr)
        command.rgfr = min(self.max_force, command.rgfr)
        command.rgwd = max(0, command.rgwd)
        command.rgwd = min(self.max_width, command.rgwd)

        # Verifying that the selected mode number is available
        if command.rctr not in [1, 8, 16]:
            self.logger.fatal(self.get_name() + ": Select the mode number from 1 (grip), 8 (stop), or 16 (grip_w_offset).")
            rclpy.shutdown()

        # Returning the modified command
        return command

    def refreshCommand(self, command : OnRobotRGOutput):
        """ Updates the command sent during the next sendCommand() call.

            Args:
                command (OnRobotRGOutput): command to be refreshed
        """

        # Limiting the value of each variable
        command = self.verifyCommand(command)

        # Initiating command as an empty list
        self.message = []

        # Building the command with each output variable
        self.message.append(command.rgfr)
        self.message.append(command.rgwd)
        self.message.append(command.rctr)

    def sendCommand(self):
        """ Sends the command to the Gripper. """

        self.client.sendCommand(self.message)
        
        if self.dummy:
            self.logger.info("Calling fake movement.")
            self.logger.info(str(self.message))
            self.fakeSystem.sendCommand(self.message)

    def restartPowerCycle(self):
        """ Restarts the power cycle of the Gripper. """
        
        self.client.restartPowerCycle()

    def getStatus(self) -> OnRobotRGInput:
        """ Requests the gripper status and return OnRobotRGInput message.

            Returns:
                message (list[int]): message including commands to be sent
        """

        # Acquiring status from the Gripper
        self.status = self.client.getStatus()
        
        if self.dummy:
            self.status = self.fakeSystem.getStatus()

        # Messaging to output
        message = OnRobotRGInput()

        # Assignning the values to their respective variables
        if isinstance(self.status, list):
            message.gfof = self.status[0]
            message.ggwd = self.status[9]
            message.gsta = self.status[10]
            message.gwdf = self.status[17]
            self.offset = message.gfof
            if self.dummy:
                self.fakeSystem.offset = message.gfof
        return message
    
    class FakeGripper:
        
        def widthToJointValue(self, width : float):
            return np.arccos(((width/2) - self.dy - self.L1 * np.cos(self.theta1)) / self.L3) - self.theta3
            
        def jointValueToWidth(self, joint_angle : float):
            return (np.cos(joint_angle + self.theta3) * self.L3 + self.dy + self.L1 * np.cos(self.theta1)) * 2

        def jointWorldCurve(self, min_width : float, max_width : float) -> tuple[np.array, np.array, np.array]:
            time = np.linspace(start=0, stop=1, num=max_width*2, dtype=np.float32)
            joint = self.widthToJointValue(min_width/10000)+(np.multiply(time, (self.widthToJointValue(max_width/10000)-self.widthToJointValue(min_width/10000))))
            width = 10000*self.jointValueToWidth(joint)
            return (time, joint, width)
        
        def interpolate(self, width):
            index = bisect_left(self.lookup[0,:], width)
            d = np.array([self.lookup[0,index], self.lookup[0,index+1]])
            c = np.divide(d, d[1]-d[0])
            return c[1]*self.lookup[1, index] + c[0]*self.lookup[1, index+1]
        
        def move(self, width, goal, control, status, stop):
            if control == 8:
                stop.value = True
                return
            elif (control in [1, 16]) and (status.value%2 == 1):
                return
            elif stop.value:
                stop.value = False
                
            error_sign = np.sign(goal - width.value)
            status.value += 1
            while (not stop.value) and (error_sign == np.sign(goal - width.value)) and (np.abs(goal - width.value) > 0.1):
                A = self.interpolate(width.value)
                B = self.interpolate(width.value+0.5*error_sign*self.force*self.force_scaling*self.dt*A)
                C = self.interpolate(width.value+0.5*error_sign*self.force*self.force_scaling*self.dt*B)
                D = self.interpolate(width.value+error_sign*self.force*self.force_scaling*self.dt*C)
                dw = error_sign*self.force*self.force_scaling*self.dt*(A+2*B+2*C+D)/6
                
                if np.sign(goal - width.value) != np.sign(goal - (width.value + dw)):
                    width.value = goal
                    error_sign *= -1
                elif width.value + dw < self.offset*2:
                    width.value = self.offset*2
                    error_sign *= -1
                elif width.value + dw > self.max_width:
                    width.value = self.max_width
                    error_sign *= -1
                else:
                    width.value += dw
                
                time.sleep(self.dt/20)
            status.value -= 1
            
        def __init__(self, max_force, max_width, offset, gtype):
            
            
            if gtype == 'rg6':
                self.L1 = 0.1394215
                self.L3 = 0.080
                self.theta1 = 1.3963
                self.theta3 = 0.93766
                self.dy = -0.0196
                
                self.force = 200
                
                '''
                Linear regression data from RG6 datasheet using points (force, speed@max_width):
                0,      0
                25,     30
                80,     90
                120,    135
                '''
                self.force_scaling = 0.1127
            
            elif gtype == 'rg2':
                self.L1 = 0.108505
                self.L3 = 0.055
                self.theta1 = 1.41371
                self.theta3 = 0.76794
                self.dy = -0.0144
                
                self.force = 20
                
                '''
                Linear regression data from RG6 datasheet using points (force, speed@max_width):
                0,      0
                3,     12.5
                20,     66.5
                40,    129.5
                '''
                self.force_scaling = 0.3259
            
            self.max_force = max_force
            self.max_width = max_width
            
            self.joint_angle = 0.0
            self.offset = offset
            
            self.manager = Manager()
            self.width = Value('d', int(self.jointValueToWidth(joint_angle=self.joint_angle)*10000))
            self.status = Value('i', 0)
            self.stop = Value(c_bool, False)
            
            self.dt = 0.001
            
            self.width_lookup = self.jointWorldCurve(0, self.max_width)
            self.lookup = np.vstack([self.width_lookup[2], np.gradient(self.width_lookup[2])])
            
            self.joint_names = ["finger_joint", "left_inner_knuckle_joint", "left_inner_finger_joint", 
                            "right_outer_knuckle_joint", "right_inner_knuckle_joint", "right_inner_finger_joint"]
            self.mimic_ratios = [1, -1, 1, -1, -1, 1]
            
        def getStatus(self):
            return list(chain([int(self.offset)], [0] * 8, [int(self.width.value), int(self.status.value)], [0] * 6, [int(self.width.value - 2*self.offset)]))
        
        def sendCommand(self, message):
            
            (self.force, target_width, control) = message
            Process(target=self.move, args=(self.width, target_width, control, self.status, self.stop)).start()
            
            
            
