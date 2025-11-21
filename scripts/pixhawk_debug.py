import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist

class PixhawkDebugTranslator(Node):
    def __init__(self):
        super().__init__('go2_debug_translator')

        # Subscribe to Pixhawk OUTPUT (rc/out)
        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.listener_callback,
            10)

        # Publisher to the Robot Dog
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- MAPPING SETTINGS ---
        # Index 0 = Servo 1 | Index 1 = Servo 2 | Index 2 = Servo 3 ...
        
        # 1. Strafe (Left Stick Left/Right - Ch4) -> Mapped to Servo 1 via RCIN4
        self.idx_strafe = 0  
        
        # 2. Forward/Back (Left Stick Up/Down - Ch3) -> Mapped to Servo 3 via Throttle
        self.idx_throttle = 2 

        # 3. Turn (Right Stick Left/Right - Ch1) -> Mapped to Servo 4 via Ground Steering
        self.idx_steering = 3 

        # Speed Limits
        self.max_linear_x  = 0.5  # Forward/Back (m/s)
        self.max_linear_y  = 0.3  # Strafe Left/Right (m/s)
        self.max_angular_z = 1.0  # Turn Speed (rad/s)

        self.get_logger().info("✅ OMNI DEBUGGER STARTED: Waiting for RC data...")

    def listener_callback(self, msg):
        if len(msg.channels) < 4:
            return

        # 1. Extract PWM Values
        pwm_strafe   = msg.channels[self.idx_strafe]   # Servo 1
        pwm_throttle = msg.channels[self.idx_throttle] # Servo 3
        pwm_steering = msg.channels[self.idx_steering] # Servo 4

        # 2. Safety Check (Disarmed/Low PWM)
        if pwm_throttle < 800 or pwm_steering < 800:
            self.publish_stop()
            return

        # 3. Map PWM to Velocity
        
        # FORWARD/BACK (Linear X) - Servo 3
        vel_x = self.map_pwm(pwm_throttle, 1000, 2000, -self.max_linear_x, self.max_linear_x)

        # STRAFE LEFT/RIGHT (Linear Y) - Servo 1
        # Left (1000) -> Positive Y (Left), Right (2000) -> Negative Y (Right)
        vel_y = self.map_pwm(pwm_strafe, 1000, 2000, self.max_linear_y, -self.max_linear_y)

        # TURN LEFT/RIGHT (Angular Z) - Servo 4
        # Left (1000) -> Positive Z (Turn Left), Right (2000) -> Negative Z (Turn Right)
        vel_z = self.map_pwm(pwm_steering, 1000, 2000, self.max_angular_z, -self.max_angular_z)

        # 4. Dashboard Print
        # We use \r to overwrite the line so it looks like a dashboard
        print(f"X (Fwd): {vel_x:5.2f} | Y (Crab): {vel_y:5.2f} | Z (Turn): {vel_z:5.2f} || PWM: {pwm_strafe}/{pwm_throttle}/{pwm_steering}  ", end="\r")

        # 5. Publish Twist
        twist = Twist()
        twist.linear.x = float(vel_x)
        twist.linear.y = float(vel_y)
        twist.angular.z = float(vel_z)
        self.publisher.publish(twist)

    def map_pwm(self, x, in_min, in_max, out_min, out_max):
        # Deadzone to prevent drift
        if 1490 < x < 1510:
            return 0.0
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_stop(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkDebugTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("\n🛑 Debugger Stopped.")

if __name__ == '__main__':
    main()