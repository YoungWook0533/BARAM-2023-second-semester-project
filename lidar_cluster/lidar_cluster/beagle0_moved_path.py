#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from roboid import *
import numpy as np

from math import *
from tf2_ros import *
from geometry_msgs.msg import *
from nav_msgs.msg import *

class MyNode(Node):

    def __init__(self):
        super().__init__("beagle_moved_path")
        self.create_timer(0.1, self.timer_callback)
        self.beagle_path_publisher = self.create_publisher(Path,"/burger/moved_path",10)

        self.subscriber_ = self.create_subscription(PoseWithCovarianceStamped,"/burger/pose",self.subscriber_callback_,10)

        self.beagle_moved_path = Path()
        self.beagle_moved_path.header.frame_id = "/map"



    def subscriber_callback_(self, msg):
        beagle_pose = PoseStamped()
        beagle_pose.pose.position.x = msg.pose.pose.position.x
        beagle_pose.pose.position.y = msg.pose.pose.position.y
        print(12345789)
        self.beagle_moved_path.poses.append(beagle_pose)



    def timer_callback(self):

        self.beagle_path_publisher.publish(self.beagle_moved_path)

def main(args = None):

    rclpy.init(args=args) # we need to contain this in first line of all program
    node = MyNode()

    rclpy.spin(node) # continue to be your node alive
                     # when we enter ctrl+c spin goes stop
    rclpy.shutdown() # in the end we need to shut down ros2 program


if __name__=="__main__":
    main()
