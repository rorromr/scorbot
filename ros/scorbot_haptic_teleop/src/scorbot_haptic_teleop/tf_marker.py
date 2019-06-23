#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
TF Interactive marker
"""

__author__ = 'Rodrigo Muñoz'

import rospy
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from visualization_msgs.msg import (InteractiveMarkerControl, InteractiveMarker,
                                    InteractiveMarkerFeedback)
from tf.broadcaster import TransformBroadcaster
from threading import Lock
import argparse

class TFMarkerServer(object):
    """TFMarkerServer"""
    def __init__(self, rate = 30.0, init_position = (0.0, 0.0, 0.0), parent_frame = "wolrd", child_frame = "tf_marker",
                 scale=0.3):
        # Marker server
        self.server = InteractiveMarkerServer('tf_marker')
        rospy.loginfo("Starting TF marker")
        rospy.loginfo("Publish transform from {} to {} at rate {}".format(parent_frame, child_frame, rate))
        rospy.loginfo("Initial position: {}".format(init_position))
        # TF broadcaster
        self.tf = TransformBroadcaster()

        # Marker pose
        self.pose_mutex = Lock()
        self.marker_position = init_position
        self.marker_orientation = (0.0, 0.0, 0.0, 1.0)
        self.scale = scale

        # Frames
        self.parent_frame = parent_frame
        self.child_frame = child_frame

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

        marker.name = 'tf_marker'
        marker.description = '6-DOF pose control'

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
        self.tf.sendTransform(self.marker_position, self.marker_orientation,
                              time, self.child_frame, self.parent_frame)
        self.pose_mutex.release()

    def marker_feedback(self, feedback):
        rospy.loginfo('Feedback from ' + feedback.marker_name)
        # Check event
        if feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
            rospy.loginfo( 'Pose changed')
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
    rospy.init_node('tf_marker')
    parser = argparse.ArgumentParser("Publish TF based on position of 6DOF marker")
    parser.add_argument('--position', nargs='+', dest='position', required=True)
    parser.add_argument('--parent_frame', dest='parent_frame', required=True)
    parser.add_argument('--child_frame', dest='child_frame', required=True)
    parser.add_argument('--rate', dest='rate', default=30.0)
    parser.add_argument('--scale', dest='scale', default=0.3)
    args = parser.parse_args()
    init_position = tuple(float(i) for i in args.position)
    marker_server = TFMarkerServer(rate=args.rate, init_position=init_position, parent_frame=args.parent_frame,
                                   child_frame=args.child_frame, scale=float(args.scale))
    rospy.spin()


if __name__ == "__main__":
    main()