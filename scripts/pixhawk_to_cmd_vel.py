import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist

class PixhawkRoverTranslator(Node):
    def __init__(self):
        super().__init__('rover_commander')

        # Subscribe to Pixhawk Output
        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- MAPPING BASED ON YOUR PARAM FILE ---
        # SERVO3_FUNCTION is 70 (Throttle) -> Index 2
        # SERVO4_FUNCTION is 26 (Steering) -> Index 3
        self.throttle_idx = 2
        self.steering_idx = 3

        self.get_logger().info("🐶 Rover Translator Started. Listening to Servo 3 & 4...")

    def listener_callback(self, msg):
        if len(msg.channels) < 4:
            return

        # 1. READ RAW PWM
        pwm_throttle = msg.channels[self.throttle_idx]
        pwm_steering = msg.channels[self.steering_idx]

        # 2. DEBUG: Uncomment this line to see what the Pixhawk is sending
        # self.get_logger().info(f"Throt: {pwm_throttle} | Steer: {pwm_steering}")

        # 3. SAFETY: If PWM is near 0 (Disarmed/Safety Switch), stop.
        if pwm_throttle < 800: 
            self.publish_stop()
            return

        # 4. MAP THROTTLE (Channel 3)
        # 1500 = Stop. >1500 = Forward. <1500 = Backward.
        # Max speed 0.5 m/s
        vel_x = self.map_pwm(pwm_throttle, 1000, 2000, -0.5, 0.5)

        # 5. MAP STEERING (Channel 4)
        # 1500 = Straight. <1500 = Left Turn (+). >1500 = Right Turn (-).
        # We invert the output because ROS usually expects + for Left.
        vel_z = self.map_pwm(pwm_steering, 1000, 2000, 1.0, -1.0) 

        # 6. PUBLISH
        twist = Twist()
        twist.linear.x = float(vel_x)
        twist.angular.z = float(vel_z)
        self.publisher.publish(twist)

    def map_pwm(self, x, in_min, in_max, out_min, out_max):
        # Deadzone of 20 PWM to prevent drifting
        if 1490 < x < 1510: return 0.0
        
        # Clamp and Map
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_stop(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkRoverTranslator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()