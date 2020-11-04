#!/usr/bin/env python2

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import random
import numpy as np


class WaypointFeed(object):

    def __init__(self):
        print('[INFO] Initializing object...')
        # COMBAK: temporarily setting queue_size to 1
        self.waypoint_pub = rospy.Publisher('/move_base_simple/goal',
                                            PoseStamped,
                                            queue_size=1)
        sub_robot = rospy.Subscriber("/visual_slam/odom", Odometry, self.update_status, queue_size=1)
        # Getting the x and y dimensions of the map to use for the random
        # waypoint
        self.map_x_size = rospy.get_param("/pcl_render_node/map/x_size", 40.0)
        self.map_y_size = rospy.get_param("/pcl_render_node/map/y_size", 40.0)
        self.prev_x, self.prev_y = None, None

    def update_status(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def publish_waypoint(self):
        waypoint_msg = PoseStamped()
        waypoint_msg.header.seq = 1
        waypoint_msg.header.stamp = rospy.Time.now()
        waypoint_msg.header.frame_id = "world"
        # Randomly setting a desired x and y position
        while True:
            cur_x = random.uniform(self.map_x_size/-2, self.map_x_size/2)
            cur_y = random.uniform(self.map_y_size/-2, self.map_y_size/2)
            if self.prev_x is None:
                break
            if cur_x * self.prev_x < 0 or cur_y * self.prev_y < 0:
                break
        waypoint_msg.pose.position.x = self.prev_x = cur_x
        waypoint_msg.pose.position.y = self.prev_y = cur_y
        waypoint_msg.pose.position.z = 0
        # COMBAK: Set w to 1.0 because that's what simply clicking on a point
        # did. Dragging when setting 2D Nav Goal changes z and w but that
        # doesn't seem to change the behavior of the drone
        waypoint_msg.pose.orientation.w = 1.0
        # Printing for debugging purposes
        print(waypoint_msg)
        self.waypoint_pub.publish(waypoint_msg)
        print('[INFO] Waypoint published')


def main():
    print('[INFO] Running file')
    rospy.init_node('waypoint_feed_node', anonymous=True)
    waypoint_feed_object = WaypointFeed()
    # TODO: See if there is a way to make it repeat every time EXEC_STATE =
    # WAIT_TARGET instead of a fixed time
    rospy.sleep(5)
    waypoint_feed_object.publish_waypoint()
    start = rospy.get_time()
    while not rospy.is_shutdown() and rospy.get_time() - start < 60 * 15:
        dist = (waypoint_feed_object.x - waypoint_feed_object.prev_x) ** 2 + \
               (waypoint_feed_object.y - waypoint_feed_object.prev_y) ** 2
        dist = np.sqrt(dist)
        if dist < 5:
            waypoint_feed_object.publish_waypoint()
    print("Shutting down")
    print('[INFO] Finishing running file')


if __name__ == '__main__':
    main()