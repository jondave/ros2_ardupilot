import rclpy
from rclpy.node import Node
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

class MissionTrigger(Node):
    def __init__(self):
        super().__init__('mission_trigger')
        
        self.state_sub = self.create_subscription(State, 'mavros/state', self.state_cb, 10)
        self.arming_client = self.create_client(CommandBool, 'mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, 'mavros/set_mode')
        
        self.current_state = State()

    def state_cb(self, msg):
        self.current_state = msg

    def start_mission(self):
        # 1. ARM the robot
        if not self.current_state.armed:
            req = CommandBool.Request()
            req.value = True
            self.arming_client.call_async(req)
            self.get_logger().info("Arming robot...")

        # 2. Switch to AUTO mode (starts the waypoint mission)
        # "AUTO" tells Pixhawk to look at the uploaded mission file
        set_mode = SetMode.Request()
        set_mode.custom_mode = 'AUTO' 
        self.mode_client.call_async(set_mode)
        self.get_logger().info("Switching to AUTO mode...")

def main():
    rclpy.init()
    node = MissionTrigger()
    
    # Simple logic: Wait for connection then start
    while not node.current_state.connected:
        rclpy.spin_once(node)
    
    node.start_mission()
    rclpy.spin(node)