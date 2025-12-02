import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn, RCOut, State

class SystemCheck(Node):
    def __init__(self):
        super().__init__('system_check')
        
        # Subscribe to all critical topics
        self.sub_rc_in  = self.create_subscription(RCIn, '/mavros/rc/in', self.cb_rc_in, 10)
        self.sub_rc_out = self.create_subscription(RCOut, '/mavros/rc/out', self.cb_rc_out, 10)
        self.sub_state  = self.create_subscription(State, '/mavros/state', self.cb_state, 10)
        
        self.rc_in_data = []
        self.rc_out_data = []
        self.armed = False
        self.connected = False
        self.mode = "UNKNOWN"

        # Create a timer to print status every 0.5 seconds
        self.timer = self.create_timer(0.5, self.print_status)
        print("🔍 STARTING DIAGNOSTIC... Wiggle your sticks!")

    def cb_state(self, msg):
        self.armed = msg.armed
        self.connected = msg.connected
        self.mode = msg.mode

    def cb_rc_in(self, msg):
        self.rc_in_data = msg.channels

    def cb_rc_out(self, msg):
        self.rc_out_data = msg.channels

    def print_status(self):
        # 1. Check Connection
        if not self.connected:
            print("❌ MAVROS: NOT CONNECTED (Check USB/wiring to Pixhawk)", end='\r')
            return

        # 2. Check Input (Remote -> Pixhawk)
        if len(self.rc_in_data) < 4:
            rc_str = "WAITING FOR REMOTE..."
        else:
            # Show stick values
            rc_str = f"Sticks: Ch1={self.rc_in_data[0]} Ch2={self.rc_in_data[1]} Ch3={self.rc_in_data[2]} Ch4={self.rc_in_data[3]}"

        # 3. Check Output (Pixhawk -> Robot)
        if len(self.rc_out_data) < 2:
            out_str = "NO OUTPUT (Pixhawk not sending commands)"
        else:
            out_str = f"Servo Out: Ch1={self.rc_out_data[0]} Ch2={self.rc_out_data[1]}"

        # 4. Print Report
        status_icon = "🟢" if self.armed else "🔴"
        print(f"{status_icon} [MODE: {self.mode}] {rc_str} || {out_str}            ", end='\r')

def main(args=None):
    rclpy.init(args=args)
    node = SystemCheck()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()