import rclpy
from rclpy.node import Node
from mavros_msgs.msg import GlobalPositionTarget
import time

class GPSWaypointSender(Node):
    def __init__(self):
        super().__init__('gps_waypoint_sender')
        
        # Publisher to the Global Setpoint Raw topic
        self.publisher = self.create_publisher(
            GlobalPositionTarget, 
            '/mavros/setpoint_raw/global', 
            10
        )
        
        # Target Coordinates (from your request)
        self.target_lat = 53.2679504
        self.target_lon = -0.5272549
        self.target_alt = 0.0  # Rovers generally ignore altitude, but good to set 0
        
        self.timer = self.create_timer(1.0, self.send_waypoint)
        self.get_logger().info(f"🚀 Sending Waypoint: {self.target_lat}, {self.target_lon}")

    def send_waypoint(self):
        msg = GlobalPositionTarget()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.coordinate_frame = GlobalPositionTarget.FRAME_GLOBAL_INT_DOC
        
        # Type Mask: Ignore Velocity and Acceleration, only use Position
        # We only want to control X (Lat) and Y (Lon)
        # The mask ignores bits that are SET (1 = ignore). 
        # We keep 0 for position bits.
        msg.type_mask = (
            GlobalPositionTarget.IGNORE_VX + 
            GlobalPositionTarget.IGNORE_VY + 
            GlobalPositionTarget.IGNORE_VZ + 
            GlobalPositionTarget.IGNORE_AFX + 
            GlobalPositionTarget.IGNORE_AFY + 
            GlobalPositionTarget.IGNORE_AFZ + 
            GlobalPositionTarget.IGNORE_YAW_RATE
        )

        msg.latitude = self.target_lat
        msg.longitude = self.target_lon
        msg.altitude = self.target_alt
        
        # Optional: Set a specific Heading (Yaw) if you want it to face a direction
        # msg.yaw = 0.0 
        
        self.publisher.publish(msg)
        # self.get_logger().info("Target sent...")

def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointSender()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()