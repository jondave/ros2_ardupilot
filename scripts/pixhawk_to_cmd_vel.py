import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist

class PixhawkDogTranslator(Node):
    def __init__(self):
        super().__init__('dog_commander')

        # Subscribe to Pixhawk Output (Safe, Armed signals)
        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.listener_callback,
            10)

        # Publish to the Dog's Velocity Command
        # CHANGE '/cmd_vel' if your dog uses a different topic (e.g. '/champ/cmd_vel')
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- CONFIGURATION ---
        # Index in the 'channels' list (ArduPilot Ch 1 = Index 0)
        self.strafe_idx   = 0  # Channel 1 (Right Stick Left/Right) -> Linear Y
        self.forward_idx  = 1  # Channel 2 (Right Stick Up/Down)    -> Linear X
        self.turn_idx     = 3  # Channel 4 (Left Stick Left/Right)  -> Angular Z
        
        # Max Speeds
        self.max_speed_x = 0.5  # m/s Forward
        self.max_speed_y = 0.3  # m/s Strafe
        self.max_turn    = 1.0  # rad/s Turn

        self.get_logger().info("🐶 Dog Commander Started! Waiting for Pixhawk...")

    def listener_callback(self, msg):
        if len(msg.channels) < 4:
            return

        # 1. Get PWM Values
        pwm_strafe  = msg.channels[self.strafe_idx]
        pwm_forward = msg.channels[self.forward_idx]
        pwm_turn    = msg.channels[self.turn_idx]

        # 2. Map PWM (1000-2000) to Velocity (Negative <-> Positive)
        # Note: We reverse Forward (Index 1) because usually Pitch Down (low PWM) is Forward
        vel_x = self.map_pwm(pwm_forward, 1000, 2000, self.max_speed_x, -self.max_speed_x)
        vel_y = self.map_pwm(pwm_strafe,  1000, 2000, -self.max_speed_y, self.max_speed_y)
        vel_z = self.map_pwm(pwm_turn,    1000, 2000, self.max_turn, -self.max_turn)

        # 3. Publish Twist
        twist = Twist()
        twist.linear.x = vel_x  # Forward/Back
        twist.linear.y = vel_y  # Left/Right Step
        twist.angular.z = vel_z # Turn
        
        self.publisher.publish(twist)

    def map_pwm(self, x, in_min, in_max, out_min, out_max):
        # Deadzone (1480-1520) so the dog stands still when sticks are centered
        if 1480 < x < 1520: return 0.0
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkDogTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()