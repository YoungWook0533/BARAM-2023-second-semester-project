# import rclpy
# from rclpy.qos import QoSProfile
# from sensor_msgs.msg import LaserScan, PointCloud
# from geometry_msgs.msg import Point32
# import math
# from rclpy.node import Node

# def create_point_cloud(points, frame_id='burger_1/base_link'):
#     point_cloud_msg = PointCloud()
#     point_cloud_msg.header.frame_id = frame_id

#     for point in points:
#         p = Point32()
#         p.x = point[0]
#         p.y = point[1]
#         point_cloud_msg.points.append(p)

#     return point_cloud_msg

# def callback(msg, publisher, node):
#     i = 0
#     infinity = float('inf')

#     # Following loop to find out the first point while starting from 0 degrees
#     for j in range(360):
#         if msg.ranges[j] != infinity:
#             i = j
#             break

#     # Initializing the first point i.e. x0 and y0
#     theta = msg.angle_min + i * msg.angle_increment
#     x = msg.ranges[i] * math.cos(theta)
#     y = msg.ranges[i] * math.sin(theta)
#     i += 1
#     points = []

#     # List to store the smallest cluster
#     smallest_cluster = []
#     smallest_cluster_size = 20

#     # Loop to compare the distance between subsequent points and find out the distance between them
#     for j in range(i, 360):
#         if msg.ranges[j] != infinity:
#             theta = msg.angle_min + j * msg.angle_increment
#             x1 = msg.ranges[j] * math.cos(theta)
#             y1 = msg.ranges[j] * math.sin(theta)
#             d = math.sqrt(pow((x1 - x), 2) + pow((y1 - y), 2))

#             if d < 0.25:  # Threshold distance taken as 0.25
#                 points.append((x1, y1))
#             else:
#                 if len(points) < smallest_cluster_size:
#                     smallest_cluster = points
#                     smallest_cluster_size = len(smallest_cluster)
#                 points = []

#             x = x1
#             y = y1

#     # Publish the smallest cluster as a PointCloud message
#     if smallest_cluster:
#         point_cloud_msg = create_point_cloud(smallest_cluster)
#         publisher.publish(point_cloud_msg)

# def main(args=None):
#     rclpy.init(args=args)
#     node = rclpy.create_node("PointCloudCluster")

#     # Define the QoS profile for the subscription
#     qos_profile = QoSProfile(depth=10)

#     # Create a publisher for the smallest cluster
#     publisher = node.create_publisher(PointCloud, 'smallest_cluster', 10)

#     # Create a subscription with the specified QoS profile
#     sub = node.create_subscription(LaserScan, 'burger_1/scan', lambda msg: callback(msg, publisher, node), qos_profile)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
#--------------------------------------------------------------------------------------------------------------------------------
import rclpy
from rclpy.qos import QoSProfile
from sensor_msgs.msg import LaserScan, PointCloud
from geometry_msgs.msg import Point32
import pandas as pd
import numpy as np
from sklearn.cluster import DBSCAN
from sklearn.preprocessing import StandardScaler
from rclpy.node import Node

def create_point_cloud(points, frame_id='burger_1/base_link'):
    point_cloud_msg = PointCloud()
    point_cloud_msg.header.frame_id = frame_id

    for point in points:
        p = Point32()
        p.x = point[0]
        p.y = point[1]
        point_cloud_msg.points.append(p)

    return point_cloud_msg

def dbscan_clustering(points):
    scaler = StandardScaler()
    df_scaler = pd.DataFrame(scaler.fit_transform(points), columns=['x', 'y'])

    model = DBSCAN(eps=0.25, min_samples=3)
    model.fit(df_scaler)
    df_scaler['label'] = model.labels_

    points_with_labels = pd.concat([points, df_scaler['label']], axis=1)
    n_labels = df_scaler['label'].nunique()

    return points_with_labels, n_labels

def callback(msg, publisher, node):
    infinity = float('inf')
    points = []

    for j in range(360):
        if msg.ranges[j] != infinity:
            theta = msg.angle_min + j * msg.angle_increment
            x = msg.ranges[j] * np.cos(theta)
            y = msg.ranges[j] * np.sin(theta)
            points.append((x, y))

    if points:
        points_with_labels, n_labels = dbscan_clustering(pd.DataFrame(points, columns=['x', 'y']))
        smallest_cluster = get_smallest_cluster(points_with_labels, n_labels)

        if smallest_cluster and len(smallest_cluster) <=10:
            point_cloud_msg = create_point_cloud(smallest_cluster)
            publisher.publish(point_cloud_msg)

            print(f"Number of clusters: {n_labels}")
            print(f"Number of points in the published cluster: {len(smallest_cluster)}")

def get_smallest_cluster(points_with_labels, n_labels):
    min_size = float('inf')
    smallest_cluster = []

    for i in range(n_labels):
        current_cluster = points_with_labels[points_with_labels['label'] == i][['x', 'y']]
        current_size = len(current_cluster)

        if current_size > 0 and current_size < min_size:
            smallest_cluster = current_cluster.values.tolist()
            min_size = current_size

    return smallest_cluster

def main(args=None):
    rclpy.init(args=args)
    node = rclpy.create_node("PointCloudCluster")

    # Define the QoS profile for the subscription
    qos_profile = QoSProfile(depth=10)

    # Create a publisher for the smallest cluster
    publisher = node.create_publisher(PointCloud, 'smallest_cluster', 10)

    # Create a subscription with the specified QoS profile
    sub = node.create_subscription(LaserScan, 'burger_1/scan', lambda msg: callback(msg, publisher, node), qos_profile)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
#--------------------------------------------------------------------------------------------------------------------------
# import rclpy
# from rclpy.qos import QoSProfile
# from sensor_msgs.msg import LaserScan, PointField
# from std_msgs.msg import Header
# from sensor_msgs.msg import PointCloud2
# import numpy as np
# import pandas as pd
# from sklearn.cluster import DBSCAN
# from sklearn.preprocessing import StandardScaler
# from rclpy.node import Node

# def create_point_cloud(points, frame_id='burger_1/base_link'):
#     # Extract x, y coordinates from the points
#     points_array = np.array(points)
#     x = points_array[:, 0]
#     y = points_array[:, 1]

#     # Create a PointCloud2 message
#     header = Header()
#     header.frame_id = frame_id

#     fields = [
#         PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
#         PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
#     ]

#     # Convert x, y coordinates to a list of tuples
#     points_list = [(float(x[i]), float(y[i]), 0.0) for i in range(len(x))]

#     # Create the PointCloud2 message
#     point_cloud_msg = PointCloud2()
#     point_cloud_msg.header = header
#     point_cloud_msg.fields = fields
#     point_cloud_msg.height = 1
#     point_cloud_msg.width = len(points)
#     point_cloud_msg.is_bigendian = False
#     point_cloud_msg.point_step = 16
#     point_cloud_msg.row_step = point_cloud_msg.point_step * point_cloud_msg.width
#     point_cloud_msg.is_dense = False
#     point_cloud_msg.data = np.array(points_list, dtype=np.float32).tobytes()

#     return point_cloud_msg

# def dbscan_clustering(points):
#     scaler = StandardScaler()
#     df_scaler = pd.DataFrame(scaler.fit_transform(points), columns=['x', 'y'])

#     model = DBSCAN(eps=0.25, min_samples=3)
#     model.fit(df_scaler)
#     df_scaler['label'] = model.labels_

#     points_with_labels = pd.concat([points, df_scaler['label']], axis=1)
#     n_labels = df_scaler['label'].nunique()

#     return points_with_labels, n_labels

# def callback(msg, publisher, node):
#     infinity = float('inf')
#     points = []

#     for j in range(360):
#         if msg.ranges[j] != infinity:
#             theta = msg.angle_min + j * msg.angle_increment
#             x = msg.ranges[j] * np.cos(theta)
#             y = msg.ranges[j] * np.sin(theta)
#             points.append((x, y))

#     if points:
#         points_with_labels, n_labels = dbscan_clustering(pd.DataFrame(points, columns=['x', 'y']))
#         smallest_cluster = get_smallest_cluster(points_with_labels, n_labels)

#         if smallest_cluster:
#             point_cloud_msg = create_point_cloud(smallest_cluster)
#             publisher.publish(point_cloud_msg)

#             print(f"Number of clusters: {n_labels}")
#             print(f"Number of points in the publishing cluster: {len(smallest_cluster)}")

# def get_smallest_cluster(points_with_labels, n_labels):
#     min_size = float('inf')
#     smallest_cluster = []

#     for i in range(n_labels):
#         current_cluster = points_with_labels[points_with_labels['label'] == i][['x', 'y']]
#         current_size = len(current_cluster)

#         if current_size > 0 and current_size < min_size:
#             smallest_cluster = current_cluster.values.tolist()
#             min_size = current_size

#     return smallest_cluster

# def main(args=None):
#     rclpy.init(args=args)
#     node = rclpy.create_node("PointCloudCluster")

#     # Define the QoS profile for the subscription
#     qos_profile = QoSProfile(depth=10)

#     # Create a publisher for the smallest cluster (PointCloud2)
#     publisher = node.create_publisher(PointCloud2, 'smallest_cluster', 10)

#     # Create a subscription with the specified QoS profile
#     sub = node.create_subscription(LaserScan, 'burger_1/scan', lambda msg: callback(msg, publisher, node), qos_profile)

#     try:
#         rclpy.spin(node)
#     except KeyboardInterrupt:
#         pass

#     node.destroy_node()
#     rclpy.shutdown()

# if __name__ == '__main__':
#     main()
