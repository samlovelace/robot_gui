import rclpy
import time
from ros_node import RosNode, AbvState
from shared_state import SharedState
from gui import GUI

import threading

def main():
    
    rclpy.init()
    shared_state = SharedState()
    node = RosNode(shared_state)
    gui = GUI(shared_state)
    
    node.create_subscription(AbvState, "/abv/state", node.state_callback, 10)
    loop_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    loop_thread.start()
    
    gui.run()
        
    rclpy.shutdown()

if __name__ == "__main__":
    main()
