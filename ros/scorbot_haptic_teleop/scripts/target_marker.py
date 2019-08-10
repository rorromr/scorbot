#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TF Interactive marker
"""

__author__ = "Rodrigo Muñoz"

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (InteractiveMarkerControl, InteractiveMarker,
                                    InteractiveMarkerFeedback)
from geometry_msgs.msg import PoseStamped
from threading import Lock
import argparse

class TargetMarkerServer(object):
    """TargetMarkerServer"""
    def __init__(self, rate = 30.0, init_position = (0.0, 0.0, 0.0), parent_frame = "world", scale=0.3):
        # Marker server
        self.server = InteractiveMarkerServer("target_marker")
        rospy.loginfo("Starting target marker")
        rospy.loginfo("Publish pose from frame_id {} at rate {}".format(parent_frame, rate))
        rospy.loginfo("Initial position: {}".format(init_position))
        # Marker pose
        self.pose_mutex = Lock()
        self.marker_position = init_position
        self.marker_orientation = (0.0, 0.0, 0.0, 1.0)
        self.scale = scale
        # Frames
        self.parent_frame = parent_frame
        # Pose publisher
        self.pub = rospy.Publisher('marker_pose', PoseStamped, queue_size=1)
        # Add marker
        self.add_6DOF()
        # Timer for TF broadcaster
        self.rate = rate
        rospy.Timer(rospy.Duration(1.0/self.rate), self.publish_transform)

    def add_6DOF(self):
        marker = InteractiveMarker()
        marker.header.frame_id = self.parent_frame
        marker.pose.position.x,marker.pose.position.y,marker.pose.position.z = self.marker_position
        marker.pose.orientation.x,marker.pose.orientation.y,marker.pose.orientation.z,marker.pose.orientation.w = self.marker_orientation
        marker.scale = self.scale

        marker.name = "target_marker"
        marker.description = ""

        # X axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "rotate_x"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # X axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Y axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_y"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Y axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Z axis rotation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        marker.controls.append(control)
        # Z axis traslation
        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        marker.controls.append(control)
        # Add marker to server
        self.server.insert(marker, self.marker_feedback)
        self.server.applyChanges()

    def publish_transform(self, timer_event):
        time = rospy.Time.now() + rospy.Duration(1.0/self.rate)
        self.pose_mutex.acquire()
        current_pose = PoseStamped()
        current_pose.header.stamp = rospy.Time.now()
        current_pose.header.frame_id = self.parent_frame
        current_pose.pose.position.x, current_pose.pose.position.y, current_pose.pose.position.z = self.marker_position
        current_pose.pose.orientation.x, current_pose.pose.orientation.y, current_pose.pose.orientation.z, \
            current_pose.pose.orientation.w = self.marker_orientation
        self.pub.publish(current_pose)
        self.pose_mutex.release()

    def marker_feedback(self, feedback):
        rospy.logdebug("Feedback from " + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo("Pose changed to {}".format(feedback.pose.position))
            # Update marker position
            self.pose_mutex.acquire()
            self.marker_position = (feedback.pose.position.x,
                                    feedback.pose.position.y, feedback.pose.position.z)
            # Update marker orientation
            self.marker_orientation = (feedback.pose.orientation.x,
                                       feedback.pose.orientation.y, feedback.pose.orientation.z,
                                       feedback.pose.orientation.w)
            self.pose_mutex.release()


def main():
    rospy.init_node("target_marker")
    parser = argparse.ArgumentParser("Publish Pose stamped message based on position of 6DOF marker")
    parser.add_argument("--position", nargs="+", dest="position", required=True)
    parser.add_argument("--parent_frame", dest="parent_frame", required=True)
    parser.add_argument("--rate", dest="rate", default=30.0)
    parser.add_argument("--scale", dest="scale", default=0.3)
    args = parser.parse_args(rospy.myargv()[1:]) # Male argparse compatible with rospy
    init_position = tuple(float(i) for i in args.position)
    marker_server = TargetMarkerServer(rate=args.rate, init_position=init_position, parent_frame=args.parent_frame, scale=float(args.scale))
    rospy.spin()


if __name__ == "__main__":
    main()