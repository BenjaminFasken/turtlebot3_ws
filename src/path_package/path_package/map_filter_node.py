import rclpy
from rclpy.node import Node
import numpy as np
import cv2
from nav_msgs.msg import OccupancyGrid, Path
from geometry_msgs.msg import PoseStamped

class MapFilterNode(Node):
    def __init__(self):
        super().__init__('map_filter_node')
        # Subscribers
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/map', self.map_callback, 10)
        self.path_sub = self.create_subscription(
            Path, '/path', self.path_callback, 10)
        # Publisher
        self.map_pub = self.create_publisher(
            OccupancyGrid, '/filtered_map', 10)
        # Storage
        self.latest_map = None
        self.latest_path = None

    def map_callback(self, msg):
        """Store the latest traversability map."""
        self.latest_map = msg
        self.get_logger().info('Received new map')

    def path_callback(self, msg):
        """Process the path and generate a filtered map."""
        self.latest_path = msg
        if self.latest_map is None:
            self.get_logger().warn('No map received yet')
            return
        self.generate_filtered_map()

    def generate_filtered_map(self):
        """Create a new map based on the path and publish it."""
        map_msg = self.latest_map
        path_msg = self.latest_path

        # Map properties
        width = map_msg.info.width
        height = map_msg.info.height
        resolution = map_msg.info.resolution
        origin_x = map_msg.info.origin.position.x
        origin_y = map_msg.info.origin.position.y

        # Convert path poses to map coordinates
        points = []
        for pose in path_msg.poses:
            x = pose.pose.position.x + 2.032197628484078
            y = pose.pose.position.y + 0.5129117160870764
            col = int((x - origin_x) / resolution)
            row = int((y - origin_y) / resolution)
            points.append((col, row))

        # Create a blank mask
        mask = np.zeros((height, width), dtype=np.uint8)

        # Draw the path on the mask
        if len(points) == 0:
            # Empty path: mask remains all zeros
            pass
        elif len(points) == 1:
            # Single point: draw a circle
            cv2.circle(mask, points[0], radius=int(1 / resolution),
                       color=255, thickness=-1)
        else:
            # Multiple points: draw lines between consecutive poses
            thickness = max(1, int(2 / resolution))  # At least 1 cell thick
            for i in range(len(points) - 1):
                cv2.line(mask, points[i], points[i + 1],
                         color=255, thickness=thickness)

        # Reshape original map data to 2D
        original_data = np.array(map_msg.data, dtype=np.int8).reshape((height, width))

        # Create new map data: keep original where mask > 0, else -1
        new_data_2d = np.where(mask > 0, original_data, -1)

        # Prepare the new map message
        new_map = OccupancyGrid()
        new_map.header = map_msg.header
        new_map.info = map_msg.info
        new_map.data = new_data_2d.flatten().tolist()

        # Publish the filtered map
        self.map_pub.publish(new_map)
        self.get_logger().info('Published filtered map')

def main(args=None):
    rclpy.init(args=args)
    node = MapFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()