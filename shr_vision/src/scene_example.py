#! /usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_commander
import tf2_geometry_msgs
from std_srvs.srv import SetBool, SetBoolRequest, SetBoolResponse
import yaml
from tf.transformations import quaternion_from_euler
import geometry_msgs.msg
import math

tau = 2 * math.pi

def create_pose(x, y, z, roll, pitch, yaw):
    pose = geometry_msgs.msg.PoseStamped()
    pose.header.frame_id = "world"
    quat_tf = quaternion_from_euler(roll, pitch, yaw)
    orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    pose.pose.orientation = orientation
    pose.pose.position.x = x
    pose.pose.position.y = y
    pose.pose.position.z = z
    return pose

if __name__ == '__main__':
    rospy.init_node('scene_example')
    scene = moveit_commander.PlanningSceneInterface()

    rate = rospy.Rate(1)
    rate.sleep()

    scene.remove_attached_object('vx300s/ee_gripper_link')
    scene.remove_world_object()
    
    scene.add_box("cube", create_pose(.3, .2, .03, 0, 0, tau/8), size=(.06, 0.06, .06))
    scene.add_box("box", create_pose(.3, -.2, .03, 0, 0, -tau/8), size=(0.145, 0.058, 0.044))
    scene.add_cylinder("cylinder", create_pose(.4, 0, .1, 0, 0, .1), .2, .015)

    rate.sleep()

    moveit_commander.roscpp_shutdown()