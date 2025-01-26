import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import Quaternion, Pose
import math

class ExploreAndNavigateNode(Node):
    def __init__(self):
        super().__init__('explore_and_navigate_node')

        # Subscriptions for LiDAR and map
        self.subscription_scan = self.create_subscription(
            LaserScan,
            '/scan',
            self.lidar_callback,
            10)

        self.subscription_map = self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10)

        # Action client for navigation
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        
        self.is_navigating = False
        self.previous_goals = []  
        self.current_pose = Pose()  

    def lidar_callback(self, msg):
        # Modtager LiDAR-data
        self.get_logger().info('Received LiDAR scan data')
        self.lidar_ranges = msg.ranges

        # Tjek om forhindringer er for tæt på
        safety_distance = 0.2  # Sikkerhedsafstand i meter
        obstacle_detected = any(distance < safety_distance for distance in msg.ranges if distance > 0)

        if obstacle_detected:
            self.get_logger().warn('Obstacle detected! Stopping navigation.')
            # Afbryd navigationen ved at sætte is_navigating til False
            self.is_navigating = False
            


    def map_callback(self, msg):
        self.get_logger().info('Received Map data')

        if self.is_navigating:
            return

        width = msg.info.width
        resolution = msg.info.resolution
        map_data = msg.data

        for i in range(len(map_data)):
            if map_data[i] == -1:  # Ukendt område
                neighbors = [
                    i - 1, i + 1,  # Venstre og højre
                    i - width, i + width  # Over og under
                ]
                for neighbor in neighbors:
                    if 0 <= neighbor < len(map_data) and map_data[neighbor] == 0:  # Fri celle
                        candidate_x = (neighbor % width) * resolution + msg.info.origin.position.x
                        candidate_y = (neighbor // width) * resolution + msg.info.origin.position.y

                        if (candidate_x, candidate_y) not in self.previous_goals:
                            self.previous_goals.append((candidate_x, candidate_y))
                            self.send_goal(candidate_x, candidate_y, 0.0)
                            return


    def send_goal(self, x, y, theta):
        # Vente til action serveren er klar
        self._action_client.wait_for_server()

        # Definerer målet
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = x
        goal_msg.pose.pose.position.y = y
        goal_msg.pose.pose.position.z = 0.0

        # Konverterer theta (yaw) til quaternion
        goal_msg.pose.pose.orientation = self.euler_to_quaternion(0.0, 0.0, theta)

        # Sender målet til action serveren
        self.get_logger().info(f'Sending goal: x={x}, y={y}, theta={theta}')
        self.is_navigating = True
        self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback).add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f'Navigation progress: {feedback.current_pose}')

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.is_navigating = False
            return

        self.get_logger().info('Goal accepted')
        goal_handle.get_result_async().add_done_callback(self.result_callback)

    def result_callback(self, future):
        result = future.result().result
        if result:
            self.get_logger().info('Goal reached successfully')
        else:
            self.get_logger().info('Failed to reach goal')

        # Når målet er nået eller fejlet, tillad nyt mål
        self.is_navigating = False

    def euler_to_quaternion(self, roll, pitch, yaw):
        qx = math.sin(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) - math.cos(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        qy = math.cos(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2)
        qz = math.cos(roll / 2) * math.cos(pitch / 2) * math.sin(yaw / 2) - math.sin(roll / 2) * math.sin(pitch / 2) * math.cos(yaw / 2)
        qw = math.cos(roll / 2) * math.cos(pitch / 2) * math.cos(yaw / 2) + math.sin(roll / 2) * math.sin(pitch / 2) * math.sin(yaw / 2)
        return Quaternion(x=qx, y=qy, z=qz, w=qw)

def main(args=None):
    rclpy.init(args=args)
    node = ExploreAndNavigateNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
