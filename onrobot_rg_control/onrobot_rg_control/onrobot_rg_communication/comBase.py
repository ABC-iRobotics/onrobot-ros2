from abc import ABC, abstractmethod

from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.time import Time
from rclpy.node import Node
from onrobot_rg_control.OnRobotRGControllerServer import OnRobotRGNode

class OnRobotCommunicationBase(ABC):
    
    # ~~~~~~~~~~~~~ ROS parameters ~~~~~~~~~~~~ #
    logger: RcutilsLogger
    
    # ~~~~~~~~~~~ Gripper parameters ~~~~~~~~~~ #
    base: OnRobotRGNode
    
    @abstractmethod
    def __init__(self, base: Node) -> None:
        pass
        
    @abstractmethod
    def sendCommand(self, force: float, joint_angle: float, command_type: int) -> None:
        pass
        
    @abstractmethod
    def restartPowerCycle(self) -> None:
        pass
        
    @abstractmethod
    def getStatus(self) -> dict:
        pass
