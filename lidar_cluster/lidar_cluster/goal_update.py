# import rclpy
# from rclpy.node import Node
# from sensor_msgs.msg import PointCloud
# from geometry_msgs.msg import Point32, PoseStamped
# from nav2_msgs.action import NavigateToPose
# from rclpy.action import ActionClient

# class ClosestPointFollower(Node):
#     def __init__(self):
#         super().__init__('closest_point_follower')

#         self.subscription = self.create_subscription(
#             PointCloud,
#             'smallest_cluster',
#             self.point_cloud_callback,
#             qos_profile=rclpy.qos.qos_profile_sensor_data
#         )

#         self.goal_update_publisher = self.create_publisher(Point32, '/goal_update', 10)
#         self.navigate_action_client = ActionClient(self, NavigateToPose, 'burger_1/navigate_to_pose')

#         # Set the timer to regenerate the goal every 0.5 seconds
#         self.timer = self.create_timer(0.5, self.timer_callback)

#     def point_cloud_callback(self, msg):
#         closest_point = self.find_closest_point(msg)
#         self.publish_goal_update(closest_point)

#         # Continue sending new goal points
#         self.send_goal(closest_point)

#     def timer_callback(self):
#         # Regenerate the goal every 0.5 seconds
#         closest_point = Point32()

#         # Check if the distance to the origin is less than 0.3
#         if self.distance(closest_point) >= 0.3:
#             # Continue sending new goal points
#             self.send_goal(closest_point)

#     def find_closest_point(self, point_cloud_msg):
#         closest_distance = float('inf')
#         closest_point = Point32()

#         for point in point_cloud_msg.points:
#             distance = (point.x ** 2 + point.y ** 2) ** 0.5

#             if distance < closest_distance:
#                 closest_distance = distance
#                 closest_point = point

#         return closest_point

#     def distance(self, point):
#         return (point.x ** 2 + point.y ** 2) ** 0.5

#     def publish_goal_update(self, closest_point):
#         self.get_logger().info(f"Publishing goal update: ({closest_point.x}, {closest_point.y})")
#         self.goal_update_publisher.publish(closest_point)

#     def send_goal(self, goal_point):
#         goal_msg = PoseStamped()
#         goal_msg.header.frame_id = 'map'
#         goal_msg.pose.position.x = goal_point.x
#         goal_msg.pose.position.y = goal_point.y
#         goal_msg.pose.orientation.w = 1.0

#         goal_action = NavigateToPose.Goal()
#         goal_action.pose = goal_msg

#         self.navigate_action_client.wait_for_server()
#         self.send_goal_future = self.navigate_action_client.send_goal_async(goal_action)

# def main(args=None):
#     rclpy.init(args=args)
#     node = ClosestPointFollower()

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#--------------------------------------------------------------------------------------------------

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud
from geometry_msgs.msg import Point32, PoseStamped, PointStamped, Point
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from tf2_ros import Buffer, TransformListener
from tf2_geometry_msgs import do_transform_point

class ClosestPointFollower(Node):
    def __init__(self):
        super().__init__('closest_point_follower')

        self.subscription = self.create_subscription(
            PointCloud,
            'smallest_cluster',
            self.point_cloud_callback,
            qos_profile=rclpy.qos.qos_profile_sensor_data
        )

        self.goal_update_publisher = self.create_publisher(Point32, '/goal_update', 10)
        self.navigate_action_client = ActionClient(self, NavigateToPose, 'burger_1/navigate_to_pose')

        # Create a transform listener
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # Set the timer to regenerate the goal every 0.5 seconds
        self.timer = self.create_timer(0.5, self.timer_callback)

    def point_cloud_callback(self, msg):
        closest_point = self.find_closest_point(msg)
        self.publish_goal_update(closest_point)

        # Check if the distance to the origin is more than 0.3
        if self.distance(closest_point) >= 0.3:
            # Continue sending new goal points
            self.send_goal(closest_point)

    def timer_callback(self):
        # Regenerate the goal every 0.5 seconds
        closest_point = Point32()

        # Check if the distance to the origin is more than 0.3
        if self.distance(closest_point) >= 0.3:
            # Continue sending new goal points
            self.send_goal(closest_point)

    def find_closest_point(self, point_cloud_msg):
        closest_distance = float('inf')
        closest_point = Point32()

        for point in point_cloud_msg.points:
            distance = (point.x ** 2 + point.y ** 2) ** 0.5

            if distance < closest_distance:
                closest_distance = distance
                closest_point = point

        # Transform the closest point to the 'map' frame
        transformed_point = self.transform_point(closest_point, point_cloud_msg.header.frame_id, 'map')
        return transformed_point

    def distance(self, point):
        return (point.x ** 2 + point.y ** 2) ** 0.5

    def publish_goal_update(self, closest_point):
        #self.get_logger().info(f"Publishing goal update: ({closest_point.x}, {closest_point.y})")
        self.goal_update_publisher.publish(closest_point)

    def send_goal(self, goal_point):
        goal_msg = PoseStamped()
        goal_msg.header.frame_id = 'map'
        goal_msg.pose.position.x = goal_point.x
        goal_msg.pose.position.y = goal_point.y
        goal_msg.pose.orientation.w = 1.0

        goal_action = NavigateToPose.Goal()
        goal_action.pose = goal_msg

        self.navigate_action_client.wait_for_server()
        self.send_goal_future = self.navigate_action_client.send_goal_async(goal_action)

    def transform_point(self, point, source_frame, target_frame):
        try:
            point_stamped = PointStamped()
            point_stamped.header.frame_id = source_frame
            point_stamped.point.x = point.x
            point_stamped.point.y = point.y

            transformed_point = do_transform_point(point_stamped, self.tf_buffer.lookup_transform(target_frame, source_frame, rclpy.time.Time()))
            return Point32(x=transformed_point.point.x, y=transformed_point.point.y)
        except Exception as e:
            self.get_logger().warn(f"Transform failed: {e}")
            return Point32()

def main(args=None):
    rclpy.init(args=args)
    node = ClosestPointFollower()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
