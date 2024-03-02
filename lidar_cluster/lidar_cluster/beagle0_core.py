#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboid import *
import numpy as np
import time
from math import *
import tf_transformations
from tf2_ros import *
import tf2_geometry_msgs
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import *
from nav_msgs.msg import *

#beagle1 == burger_1

class MyNode(Node):

    def __init__(self):
        super().__init__("beagle_core")
        self.create_timer(0.01, self.timer_callback)
        self.publisher_ = self.create_publisher(LaserScan,"/burger/scan", 10)
        self.publisher_odom = self.create_publisher(Odometry,"/burger/odom", 10)
        self.beagle_path_publisher = self.create_publisher(Path,"/burger/moved_path",10)
        self.subscriber_ = self.create_subscription(Twist,"/burger/cmd_vel",self.subscriber_callback,10)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_static_broadcaster = StaticTransformBroadcaster(self)

        self.beagle = Beagle()

        self.value = [0.0,]
        self.nvalue = [0.0,]

        self.last_cmd_RPS_L = 0.0
        self.last_cmd_RPS_R = 0.0

        self.last_cmd_vel_L = 0.0
        self.last_cmd_vel_R = 0.0

        self.beagle_vel_L = 0.0
        self.beagle_vel_R = 0.0

        # variable for odom
        self.left_vel = 0.0
        self.right_vel = 0.0

        self.recent_x = 0.0
        self.recent_y = 0.0
        self.recent_theta = 0.0000000000000

        self.last_x = 0.0
        self.last_y = 0.0
        self.last_theta = 0.00000000

        self.wheel_radius = 0.033

        self.recent_time = time.time()
        self.last_time = time.time()

        self.dt = 0.0
        self.wheel_base = 0.09560
        self.max_rpm = 93.75

        self.quaternion = [0.0,0.0,0.0,0.0]

        self.odom = Odometry()

        self.timer_cnt = 0

        self.beagle_moved_path = Path()
        self.beagle_moved_path.header.frame_id = "/burger/odom"

    def timer_callback(self):

        self.recent_time = time.time()
        self.dt = self.recent_time - self.last_time

        #self.beagle._motion_sec(0.02,self.last_cmd_vel_L,self.last_cmd_vel_R)
        self.beagle.write(self.beagle.LEFT_WHEEL, self.beagle_vel_L)
        self.beagle.write(self.beagle.RIGHT_WHEEL, self.beagle_vel_R)

        # print(self.beagle_vel_L,self.beagle_vel_R)

        if self.timer_cnt % 19 == 0 :
            self.publish_odom()
            self.tf_publisher()
            self.publish_lidar()
            self.timer_cnt = 0
            self.beagle_path_publisher.publish(self.beagle_moved_path)
        self.update_pose()
        #self.static_map_to_odom()
        self.last_time = self.recent_time
        self.timer_cnt += 1

    def subscriber_callback(self,msg):

        # odometry를 위한 pose update를 하는 과정인데 이 부분에서 수정이 필요하다고 생각됨
        self.last_cmd_vel_L = msg.linear.x
        self.last_cmd_vel_R = msg.angular.z

        #self.left_vel = msg.linear.x * 0.00323977
        #self.right_vel = msg.angular.z * 0.00323977

        # 각 바퀴 중심의 선속도
        self.left_vel = msg.linear.x - (msg.angular.z * self.wheel_base / 2)
        self.right_vel = msg.linear.x + (msg.angular.z * self.wheel_base / 2)

        # 바퀴의 속도를 100으로 지정했을때 RPM값이 93.75 / m 이 나오기 때문에
        # (0.033 * 2 * pi) 바퀴의 원주
        # 93.75/60 = max Revolution per sec
        # 0.066 * pi * 93.75 / 60 = max linear velocity per sec
        # 0.066 * pi * command RPS = linear velocity per sec
        # 각 바퀴의 각속도 = linear velocity / (0.066 * pi)

        # 비글의 하드웨어 스펙상의 최고속도는 0.323977m/s이다 초당 대략 32.4cm 움직일 수 있음



        self.last_cmd_RPS_L = self.left_vel / self.wheel_radius
        self.last_cmd_RPS_R = self.right_vel / self.wheel_radius

        self.beagle_vel_L = self.last_cmd_RPS_L/ (self.max_rpm / 60.0 * 2 * pi) * 100.0
        self.beagle_vel_R = self.last_cmd_RPS_R/ (self.max_rpm / 60.0 * 2 * pi) * 100.0

        self.odom.twist.twist.linear.x = msg.linear.x
        self.odom.twist.twist.angular.z = msg.angular.z


    def publish_lidar(self):
        self.value = self.beagle.lidar()
        msg = LaserScan()
        msg.header.frame_id = 'burger/base_link'
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.angle_min = -3.14
        msg.angle_max = +3.14
        msg.angle_increment = 3.14/180
        msg.time_increment = 0.001
        msg.scan_time = 0.2
        msg.range_max = 5.0
        msg.range_min = 0.05
        msg.intensities = [0.0] * 360

        tmp = np.zeros((360), dtype=np.float32)

        for i in range(0,360): # 0 ~ 359
            if self.value[i] >= 0 and self.value[i] <= 2000 :
                tmp[i] = self.value[i]/1000.0
            else :
                tmp[i] = np.Inf

        # msg.ranges = list(np.float_(self.value))
        msg.ranges = tmp.tolist()

        self.publisher_.publish(msg)


    def update_pose(self):

        self.recent_x = self.last_x + self.dt*(self.left_vel + self.right_vel)*cos(self.last_theta)/2.0
        self.recent_y = self.last_y + self.dt*(self.left_vel + self.right_vel)*sin(self.last_theta)/2.0
        if (self.beagle.gyroscope_z() > 1.0 or self.beagle.gyroscope_z() < -1.0):
            self.recent_theta = self.last_theta + self.dt*self.beagle.gyroscope_z()*pi/180.0000000
            print(self.beagle.gyroscope_z() , self.recent_theta)

        self.last_x = self.recent_x
        self.last_y = self.recent_y
        self.last_theta = self.recent_theta



        beagle_pose = PoseStamped()
        beagle_pose.pose.position.x = self.last_x
        beagle_pose.pose.position.y = self.last_y

        self.beagle_moved_path.poses.append(beagle_pose)




        #print(f"odom test -> x = {self.recent_x} y = {self.recent_y} theta = {self.recent_theta}")

    def publish_odom(self):
        self.odom.header.frame_id = 'burger/odom'
        self.odom.header.stamp = self.get_clock().now().to_msg()
        self.odom.child_frame_id = 'burger/base_scan'
        self.odom.pose.pose.position.x = self.recent_x
        self.odom.pose.pose.position.y = self.recent_y

        self.quaternion = tf_transformations.quaternion_from_euler(0,0,self.recent_theta) #pi
        self.odom.pose.pose.orientation.x = self.quaternion[0]
        self.odom.pose.pose.orientation.y = self.quaternion[1]
        self.odom.pose.pose.orientation.z = self.quaternion[2]
        self.odom.pose.pose.orientation.w = self.quaternion[3]

        self.publisher_odom.publish(self.odom)

    def tf_publisher(self):

        map_to_odom = TransformStamped()
        map_to_odom.header.stamp = self.get_clock().now().to_msg()
        map_to_odom.header.frame_id = 'map'
        map_to_odom.child_frame_id = 'burger/odom'

        self.tf_broadcaster.sendTransform(map_to_odom)

        # t = TransformStamped()

        # t.header.stamp = self.get_clock().now().to_msg()
        # t.header.frame_id = 'burger_1/chassis'
        # t.child_frame_id = 'burger_1/base_scan'

        # t.transform.translation.x = self.odom.pose.pose.position.x
        # t.transform.translation.y = self.odom.pose.pose.position.y

        # t.transform.rotation.x = self.odom.pose.pose.orientation.x
        # t.transform.rotation.y = self.odom.pose.pose.orientation.y
        # t.transform.rotation.z = self.odom.pose.pose.orientation.z
        # t.transform.rotation.w = self.odom.pose.pose.orientation.w

        # self.tf_broadcaster.sendTransform(t)
        
        # Transform from odom to base_link
        odom_to_base_link = TransformStamped()
        odom_to_base_link.header.stamp = self.get_clock().now().to_msg()
        odom_to_base_link.header.frame_id = 'burger/odom'
        odom_to_base_link.child_frame_id = 'burger/base_link'
        odom_to_base_link.transform.translation.x = self.odom.pose.pose.position.x
        odom_to_base_link.transform.translation.y = self.odom.pose.pose.position.y
        odom_to_base_link.transform.translation.z = self.odom.pose.pose.position.z
        odom_to_base_link.transform.rotation.x = self.odom.pose.pose.orientation.x
        odom_to_base_link.transform.rotation.y = self.odom.pose.pose.orientation.y
        odom_to_base_link.transform.rotation.z = self.odom.pose.pose.orientation.z
        odom_to_base_link.transform.rotation.w = self.odom.pose.pose.orientation.w
        self.tf_broadcaster.sendTransform(odom_to_base_link)

        # Create a static transform message
        base_footprint_to_base_link = TransformStamped()
        base_footprint_to_base_link.header.stamp = self.get_clock().now().to_msg()
        base_footprint_to_base_link.header.frame_id = 'burger/base_link'
        base_footprint_to_base_link.child_frame_id = 'burger/base_footprint'
        base_footprint_to_base_link.transform.translation.x = 0.0
        base_footprint_to_base_link.transform.translation.y = 0.0
        base_footprint_to_base_link.transform.translation.z = 0.05  # Adjust accordingly
        base_footprint_to_base_link.transform.rotation.x = 0.0
        base_footprint_to_base_link.transform.rotation.y = 0.0
        base_footprint_to_base_link.transform.rotation.z = 0.0
        base_footprint_to_base_link.transform.rotation.w = 1.0
        self.tf_static_broadcaster.sendTransform(base_footprint_to_base_link)

        # Transform from base_link to left_wheel
        base_link_to_left_wheel = TransformStamped()
        base_link_to_left_wheel.header.stamp = self.get_clock().now().to_msg()
        base_link_to_left_wheel.header.frame_id = 'burger/base_link'
        base_link_to_left_wheel.child_frame_id = 'burger/left_wheel'
        base_link_to_left_wheel.transform.translation.x = 0.016
        base_link_to_left_wheel.transform.translation.y = 0.0481
        base_link_to_left_wheel.transform.translation.z = 0.0
        base_link_to_left_wheel.transform.rotation.x = 0.0
        base_link_to_left_wheel.transform.rotation.y = 0.0
        base_link_to_left_wheel.transform.rotation.z = sin(-pi/4.0)
        base_link_to_left_wheel.transform.rotation.w = cos(-pi/4.0)
        self.tf_broadcaster.sendTransform(base_link_to_left_wheel)

        # Transform from base_link to right_wheel
        base_link_to_right_wheel = TransformStamped()
        base_link_to_right_wheel.header.stamp = self.get_clock().now().to_msg()
        base_link_to_right_wheel.header.frame_id = 'burger/base_link'
        base_link_to_right_wheel.child_frame_id = 'burger/right_wheel'
        base_link_to_right_wheel.transform.translation.x = 0.016
        base_link_to_right_wheel.transform.translation.y = -0.0481
        base_link_to_right_wheel.transform.translation.z = 0.0
        base_link_to_right_wheel.transform.rotation.x = 0.0
        base_link_to_right_wheel.transform.rotation.y = 0.0
        base_link_to_right_wheel.transform.rotation.z = sin(pi/4.0)
        base_link_to_right_wheel.transform.rotation.w = cos(pi/4.0)
        self.tf_broadcaster.sendTransform(base_link_to_right_wheel)

        # Transform from base_link to chassis
        base_link_to_chassis = TransformStamped()
        base_link_to_chassis.header.stamp = self.get_clock().now().to_msg()
        base_link_to_chassis.header.frame_id = 'burger/base_link'
        base_link_to_chassis.child_frame_id = 'burger/chassis'
        base_link_to_chassis.transform.translation.x = -0.05
        base_link_to_chassis.transform.translation.y = 0.0
        base_link_to_chassis.transform.translation.z = 0.0
        base_link_to_chassis.transform.rotation.x = 0.0
        base_link_to_chassis.transform.rotation.y = 0.0
        base_link_to_chassis.transform.rotation.z = 0.0
        base_link_to_chassis.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(base_link_to_chassis)

        # Transform from chassis to base_scan
        chassis_to_base_scan = TransformStamped()
        chassis_to_base_scan.header.stamp = self.get_clock().now().to_msg()
        chassis_to_base_scan.header.frame_id = 'burger/chassis'
        chassis_to_base_scan.child_frame_id = 'burger/base_scan'
        chassis_to_base_scan.transform.translation.x = 0.06
        chassis_to_base_scan.transform.translation.y = 0.0
        chassis_to_base_scan.transform.translation.z = 0.05
        chassis_to_base_scan.transform.rotation.x = 0.0
        chassis_to_base_scan.transform.rotation.y = 0.0
        chassis_to_base_scan.transform.rotation.z = 0.0
        chassis_to_base_scan.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(chassis_to_base_scan)    

def main(args = None):

    rclpy.init(args=args) # we need to contain this in first line of all program
    node = MyNode()
    node.beagle.start_lidar()
    node.beagle.wait_until_lidar_ready()
    print('lidar is ready.')

    rclpy.spin(node) # continue to be your node alive
                     # when we enter ctrl+c spin goes stop
    rclpy.shutdown() # in the end we need to shut down ros2 program


if __name__=="__main__":
    main()
