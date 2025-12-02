import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist

class PixhawkWaypointDriver(Node):
    def __init__(self):
        super().__init__('go2_waypoint_driver')

        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- CONFIGURATION FROM YOUR SCREENSHOTS ---
        # Image shows:
        # Servo 1: Function "Throttle"       (Center 1500)
        # Servo 2: Function "GroundSteering" (Center 1600)
        
        self.idx_throttle = 0  # Servo 1 (Index 0 in ROS)
        self.idx_steering = 1  # Servo 2 (Index 1 in ROS)

        # --- SPEED LIMITS ---
        self.max_linear_x  = 0.5   # Max Forward Speed (m/s)
        self.max_angular_z = 1.0   # Max Turn Speed (rad/s)

        self.get_logger().info("✅ WAYPOINT DRIVER ACTIVE: Locked to Servo 1 (Thr) & Servo 2 (Str)")

    def listener_callback(self, msg):
        # We need at least channels 1 and 2
        if len(msg.channels) < 2: return

        # 1. Read Inputs directly from Pixhawk Output
        pwm_throttle  = msg.channels[self.idx_throttle] # Servo 1
        pwm_steer     = msg.channels[self.idx_steering] # Servo 2

        # 2. Safety Check (If PWM is 0, Pixhawk is off/disconnected)
        if pwm_throttle == 0:
            self.publish_stop()
            return

        # 3. Map PWM to Velocity
        
        # THROTTLE (Servo 1)
        # Screenshot: Min 1100, Trim 1500, Max 1900
        # 1900 = Forward (+X), 1100 = Reverse (-X)
        vel_x = self.map_pwm(pwm_throttle, 1100, 1900, 1500, -self.max_linear_x, self.max_linear_x)
        
        # STEERING (Servo 2)
        # Screenshot: Min 1100, **Trim 1600**, Max 1900
        # We MUST use 1600 as the center or the robot will spin constantly.
        # Standard: 1900 = Left Turn (+Z), 1100 = Right Turn (-Z)
        vel_z = self.map_pwm(pwm_steer, 1100, 1900, 1600, -self.max_angular_z, self.max_angular_z)

        # 4. No Crabbing in Waypoint Missions (Car-like steering)
        vel_y = 0.0

        # 5. Print Dashboard
        print(f"FWD: {vel_x:5.2f} | TURN: {vel_z:5.2f} || PWM_Thr: {pwm_throttle} / PWM_Str: {pwm_steer}   ", end="\r")

        # 6. Publish
        twist = Twist()
        twist.linear.x = float(vel_x)
        twist.linear.y = float(vel_y)
        twist.angular.z = float(vel_z)
        self.publisher.publish(twist)

    def map_pwm(self, x, in_min, in_max, trim_center, out_min, out_max, deadzone=10):
        """ 
        Maps PWM to Velocity handling the specific Trim Center (1600).
        """
        # 1. Apply Deadzone around the specific Trim Center
        if (trim_center - deadzone) < x < (trim_center + deadzone):
            return 0.0
            
        # 2. Clamp input
        x = max(in_min, min(x, in_max))
        
        # 3. Map (Split into two halves to handle the 1600 offset)
        if x > trim_center:
            # Upper half (e.g., 1600 to 1900)
            percentage = (x - trim_center) / (in_max - trim_center)
            return percentage * out_max
        else:
            # Lower half (e.g., 1100 to 1600)
            percentage = (trim_center - x) / (trim_center - in_min)
            return percentage * out_min

    def publish_stop(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkWaypointDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()