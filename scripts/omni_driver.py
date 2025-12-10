import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCIn, RCOut, State
from geometry_msgs.msg import Twist

class HybridOmniDriver(Node):
    def __init__(self):
        super().__init__('go2_hybrid_driver')

        # 1. Subscribe to State (To know if we are in MANUAL or AUTO)
        self.sub_state = self.create_subscription(
            State, '/mavros/state', self.cb_state, 10)

        # 2. Subscribe to STICKS (For Manual Mode)
        self.sub_rc_in = self.create_subscription(
            RCIn, '/mavros/rc/in', self.cb_rc_in, 10)

        # 3. Subscribe to SERVOS (For Auto/Mission Mode)
        self.sub_rc_out = self.create_subscription(
            RCOut, '/mavros/rc/out', self.cb_rc_out, 10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- CONFIGURATION ---
        self.current_mode = "MANUAL"
        
        # MANUAL SETTINGS (Sticks)
        # Forward/Back -> Left Stick L/R (Ch1)
        # Crab L/R      -> Left Stick U/D (Ch4)
        # Turn L/R      -> Right Stick L/R (Ch2)
        self.idx_man_fwd  = 0  # Ch1
        self.idx_man_crab = 3  # Ch4
        self.idx_man_turn = 1  # Ch2

        # AUTO SETTINGS (Pixhawk Servos)
        # Matched to your Mission Planner Setup:
        # Servo 2 = GroundSteering (Index 1)
        # Servo 3 = Throttle (Index 2)
        
        self.idx_auto_str = 1 # Servo 2 (Index 1)
        self.idx_auto_thr = 2 # Servo 3 (Index 2)

        # SPEEDS
        self.max_spd = 1.0
        self.max_turn = 1.2

        self.get_logger().info("✅ HYBRID DRIVER: Manual=Sticks | Auto=Waypoints (Servo 2/3)")

    def cb_state(self, msg):
        self.current_mode = msg.mode

    def cb_rc_in(self, msg):
        # We only use RC Inputs (Sticks) if we are in MANUAL mode
        if self.current_mode != "MANUAL":
            return
            
        if len(msg.channels) < 4: return

        # 1. Read Sticks
        pwm_fwd  = msg.channels[self.idx_man_fwd]  # Ch1
        pwm_crab = msg.channels[self.idx_man_crab] # Ch4
        pwm_turn = msg.channels[self.idx_man_turn] # Ch2

        # 2. Map Sticks to Velocity (Omni-Directional)
        # Note: These map settings work for your MANUAL mode
        vel_x = self.map_pwm(pwm_fwd, 1000, 2000, 1500, -self.max_spd, self.max_spd)
        vel_y = self.map_pwm(pwm_crab, 1000, 2000, 1500, self.max_spd, -self.max_spd) # Inverted for correct crab
        vel_z = self.map_pwm(pwm_turn, 1000, 2000, 1500, self.max_turn, -self.max_turn)

        self.publish_twist(vel_x, vel_y, vel_z, "MANUAL (Sticks)")

    def cb_rc_out(self, msg):
        # We only use RC Output (Servos) if we are in AUTO/GUIDED/RTL mode
        if self.current_mode == "MANUAL":
            return

        # Ensure we have enough channels (we need at least up to index 2 / Servo 3)
        if len(msg.channels) < 3: return

        # 1. Read Pixhawk Navigation Commands
        pwm_nav_str = msg.channels[self.idx_auto_str] # Servo 2
        pwm_nav_thr = msg.channels[self.idx_auto_thr] # Servo 3

        if pwm_nav_thr == 0: return # Safety check: If Pixhawk sends 0, don't move.

        # 2. Map Servos to Velocity (Car-Like, No Crabbing)
        
        # THROTTLE (Servo 3): Center is 1500 based on your screenshot
        vel_x = self.map_pwm(pwm_nav_thr, 1100, 1900, 1500, -self.max_spd, self.max_spd)
        
        # STEERING (Servo 2): Center is 1600 based on your screenshot
        # FIX FOR CIRCLING: Swapped signs here (self.max_turn, -self.max_turn)
        # This ensures Pixhawk "High PWM" (Right Turn) results in Negative Z (Right Turn)
        vel_z = self.map_pwm(pwm_nav_str, 1100, 1900, 1600, self.max_turn, -self.max_turn)
        
        vel_y = 0.0 

        self.publish_twist(vel_x, vel_y, vel_z, f"{self.current_mode} (Pixhawk)")

    def publish_twist(self, x, y, z, source):
        # Dashboard
        print(f"[{source}] X: {x:5.2f} | Y: {y:5.2f} | Z: {z:5.2f}          ", end="\r")
        
        t = Twist()
        t.linear.x = float(x)
        t.linear.y = float(y)
        t.angular.z = float(z)
        self.publisher.publish(t)

    def map_pwm(self, x, in_min, in_max, center, out_min, out_max):
        # Increased deadzone slightly to prevent drift
        deadzone = 30 
        
        if (center - deadzone) < x < (center + deadzone):
            return 0.0
            
        x = max(in_min, min(x, in_max))
        
        if x > center:
            return (x - center) / (in_max - center) * out_max
        else:
            return (center - x) / (center - in_min) * out_min

def main(args=None):
    rclpy.init(args=args)
    node = HybridOmniDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()