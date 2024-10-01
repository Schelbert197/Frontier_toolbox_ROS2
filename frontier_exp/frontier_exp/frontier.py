import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped
import numpy as np
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException


class FrontierExplorationNode(Node):
    def __init__(self):
        super().__init__('frontier_exploration_node')

        # Parameter to specify if we are in simulation or real robot
        self.declare_parameter('is_sim', True)
        self.is_sim = self.get_parameter(
            'is_sim').get_parameter_value().bool_value

        # Subscribe to the /map topic
        self.map_subscriber = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        # Publisher for goal pose
        self.goal_publisher = self.create_publisher(
            PoseStamped, '/goal_pose', 10)

        # Store frontiers and map information
        self.frontiers = []
        self.map_data = None
        self.map_resolution = None
        self.map_origin = None

        # tf2 buffer and listener for transforms
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

    def map_callback(self, msg: OccupancyGrid):
        self.map_data = np.array(msg.data).reshape(
            (msg.info.height, msg.info.width))
        self.map_resolution = msg.info.resolution
        self.map_origin = (msg.info.origin.position.x,
                           msg.info.origin.position.y)

        # Get the robot's current position from the transform
        robot_position = self.get_robot_position()

        if robot_position:
            self.robot_position = robot_position
            # Detect frontiers
            self.find_frontiers()

            # Remove frontiers that are no longer valid
            self.cleanup_frontiers()

            # Find and publish the nearest frontier
            self.publish_nearest_frontier()
        else:
            self.get_logger().warn("Unable to determine robot's position.")

    def get_robot_position(self):
        # Determine which transform to use based on the is_sim parameter
        base_frame = 'base_footprint' if self.is_sim else 'base_link'
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', base_frame, rclpy.time.Time())
            # Extract position from the transform
            x = transform.transform.translation.x
            y = transform.transform.translation.y
            return (x, y)
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().error(f"Error getting transform: {str(e)}")
            return None

    def find_frontiers(self):
        height, width = self.map_data.shape
        for y in range(height):
            for x in range(width):
                if self.map_data[y, x] == -1:  # Unknown cell
                    # Check if it's a frontier (bordering a free cell)
                    if self.is_frontier(x, y):
                        self.frontiers.append((x, y))

    def is_frontier(self, x, y):
        # Check neighboring cells to see if any are free (value 0)
        neighbors = [(x-1, y), (x+1, y), (x, y-1), (x, y+1)]
        for nx, ny in neighbors:
            if 0 <= nx < self.map_data.shape[1] and 0 <= ny < self.map_data.shape[0]:
                if self.map_data[ny, nx] == 0:
                    return True
        return False

    def cleanup_frontiers(self):
        # Remove frontiers that are no longer valid (explored or occupied)
        valid_frontiers = []
        for x, y in self.frontiers:
            if self.map_data[y, x] == -1:  # Still unknown
                valid_frontiers.append((x, y))
        self.frontiers = valid_frontiers

    def publish_nearest_frontier(self):
        if not self.frontiers:
            self.get_logger().info("No frontiers available.")
            return

        # Find the closest frontier to the robot
        nearest_frontier = min(
            self.frontiers, key=lambda f: self.distance_to_robot(f))

        # Convert frontier cell to world coordinates
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = "map"
        goal_pose.pose.position.x, goal_pose.pose.position.y = self.cell_to_world(
            nearest_frontier)
        goal_pose.pose.position.z = 0.0  # Flat ground
        goal_pose.pose.orientation.w = 1.0  # No specific orientation required

        # Publish the goal
        self.goal_publisher.publish(goal_pose)
        self.get_logger().info(
            f"Publishing goal at {goal_pose.pose.position.x}, {goal_pose.pose.position.y}")

    def distance_to_robot(self, frontier):
        # Compute the distance from the robot to a frontier (in grid coordinates)
        fx, fy = frontier
        rx, ry = self.robot_position
        return np.hypot(fx - rx, fy - ry)

    def cell_to_world(self, cell):
        # Convert a grid cell to world coordinates
        x, y = cell
        world_x = self.map_origin[0] + (x * self.map_resolution)
        world_y = self.map_origin[1] + (y * self.map_resolution)
        return world_x, world_y


def main(args=None):
    rclpy.init(args=args)
    node = FrontierExplorationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
