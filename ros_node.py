
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from robot_idl.msg import AbvState
from shared_state import SharedState
import time

class RosNode(Node): 
    
    def __init__(self, a_shared_state : SharedState):
        super().__init__('robot_gui')  # Initialize the Node base class
        self.subscribers = []
        self.shared_state = a_shared_state
        
    def state_callback(self, state_msg : AbvState):
        tmp = {}
        
        tmp["position"] = [state_msg.position.x, state_msg.position.y, state_msg.position.z]
        tmp["orientation"] = [state_msg.orientation.z, state_msg.orientation.y, state_msg.orientation.x]
        tmp["velocity"] = [state_msg.velocity.x, state_msg.velocity.y, state_msg.velocity.z]
        tmp["ang_vel"] = [state_msg.ang_vel.z, state_msg.ang_vel.y, state_msg.ang_vel.x]
        
        self.shared_state.set_state(tmp)
    
