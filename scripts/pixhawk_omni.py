'''
Use this pixhawk params with this python file
unitree_go2_edu_ros2_mavlink_rc_controller_omni.param
'''

import rclpy
from rclpy.node import Node
from mavros_msgs.msg import RCOut
from geometry_msgs.msg import Twist

class PixhawkOmniTranslator(Node):
    def __init__(self):
        super().__init__('go2_omni_translator')

        # Subscribe to Mavros RC Output
        self.subscription = self.create_subscription(
            RCOut,
            '/mavros/rc/out',
            self.listener_callback,
            10)

        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        # --- USER CONFIGURATION ---
        # 1. Forward/Back -> Left Stick L/R        (Channel 1)
        # 2. Crab L/R     -> Left Stick Up/Down    (Channel 4)
        # 3. Turn L/R     -> Right Stick L/R       (Channel 2)
        
        self.idx_forward = 0  # Channel 1
        self.idx_crab    = 3  # Channel 4
        self.idx_turn    = 1  # Channel 2

        # Speed Limits
        self.max_linear_x  = 0.5   # Forward Speed
        self.max_linear_y  = 0.3   # Crab Speed
        self.max_angular_z = 1.0   # Turn Speed

        self.get_logger().info("✅ OMNI DRIVER: L-Stick(X)=Drive, L-Stick(Y)=Crab(Inverted), R-Stick(X)=Turn")

    def listener_callback(self, msg):
        if len(msg.channels) < 4: return

        # 1. Read Inputs
        pwm_forward = msg.channels[self.idx_forward] # Ch 1
        pwm_crab    = msg.channels[self.idx_crab]    # Ch 4
        pwm_turn    = msg.channels[self.idx_turn]    # Ch 2

        # 2. Safety Check (Stop if RC disconnects/PWM is 0)
        if pwm_forward == 0 or pwm_crab == 0:
            self.publish_stop()
            return

        # 3. Map Raw Velocities
        
        # FORWARD (Left Stick Left/Right - Ch1)
        # Left (1000) -> Backward (-X)
        # Right (2000) -> Forward (+X)
        vel_x = self.map_pwm(pwm_forward, 1000, 2000, -self.max_linear_x, self.max_linear_x, deadzone=50)
        
        # CRAB (Left Stick Up/Down - Ch4) [DIRECTION FLIPPED]
        # Previous: Down -> Right, Up -> Left
        # NOW:      Down -> Left (+Y), Up -> Right (-Y)
        # (Swapped the last two arguments: self.max, -self.max)
        vel_y = self.map_pwm(pwm_crab, 1000, 2000, self.max_linear_y, -self.max_linear_y, deadzone=50)
        
        # TURN (Right Stick Left/Right - Ch2)
        # Left (1000) -> Turn Left (+Z)
        # Right (2000) -> Turn Right (-Z)
        vel_z = self.map_pwm(pwm_turn, 1000, 2000, self.max_angular_z, -self.max_angular_z, deadzone=50)

        # 4. Print Dashboard
        print(f"FWD: {vel_x:5.2f} | CRAB: {vel_y:5.2f} | TURN: {vel_z:5.2f} || PWM: {pwm_forward}/{pwm_crab}/{pwm_turn}   ", end="\r")

        # 5. Publish
        twist = Twist()
        twist.linear.x = float(vel_x)
        twist.linear.y = float(vel_y)
        twist.angular.z = float(vel_z)
        self.publisher.publish(twist)

    def map_pwm(self, x, in_min, in_max, out_min, out_max, deadzone=10):
        """ Maps PWM 1000-2000 to Velocity, with a center deadzone. """
        center = (in_min + in_max) / 2
        if (center - deadzone) < x < (center + deadzone):
            return 0.0
        x = max(in_min, min(x, in_max))
        return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

    def publish_stop(self):
        self.publisher.publish(Twist())

def main(args=None):
    rclpy.init(args=args)
    node = PixhawkOmniTranslator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()