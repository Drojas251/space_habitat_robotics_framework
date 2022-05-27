#! /usr/bin/env python3


# Transform a given input pose from one fixed frame to another
import rospy
from geometry_msgs.msg import Pose

from shr_interfaces.srv import Transform, TransformResponse

import tf2_ros
import tf2_geometry_msgs  # **Do not use geometry_msgs. Use this instead for PoseStamped

class TransformServer():
    def __init__(self):
        self.tf_buffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tf_buffer)
        s = rospy.Service('transform_pose', Transform, self.handle_transform)

    def handle_transform(self, req):

        # **Assuming /tf2 topic is being broadcasted
        

        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = req.pose
        pose_stamped.header.frame_id = req.from_frame

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tf_buffer.transform(pose_stamped, req.to_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('oh no')

        res = TransformResponse()
        res.pose = output_pose_stamped.pose
        return res


# Test Case




if __name__ == "__main__":
    rospy.init_node("transform_server")
    transform_server = TransformServer()
    rospy.spin()