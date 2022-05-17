#! /usr/bin/env python3

import rospy
import moveit_commander
import numpy as np

from tf.transformations import quaternion_from_euler
import math

import moveit_msgs.msg
import trajectory_msgs.msg
import geometry_msgs.msg

from shr_interfaces.srv import Grasps, GraspsRequest
from std_srvs.srv import Empty
from topic_tools.srv import MuxSelect, MuxSelectRequest


tau = 2 * math.pi 


class ManipulationPrimitives:
    def __init__(self):
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("interbotix_arm")
        self.gripper = moveit_commander.MoveGroupCommander("interbotix_gripper")
        
        self.arm.allow_replanning(True)
        self.arm.set_max_velocity_scaling_factor(1.0)
        self.arm.set_max_acceleration_scaling_factor(1.0)

        self.grasp_planning_service = rospy.ServiceProxy('grasp_planning_service', Grasps)

        self.finger_max_in = .025
        self.finger_max_out = .0285
        self.max_gripper_width = .08

        self.attached_object = None

    def move_to_pose_msg(self, pose):
        # Moves to a defined Pose
        status = False

        self.arm.set_pose_target(pose)

        error_code_val, plan, planning_time, error_code = self.arm.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        error_code_val = self.arm.execute(plan)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        status = True
        return status
    
    def move_to_pose(self,x, y, z, roll, pitch, yaw):
        # Moves to a defined Pose
        status = False

        gripper_pose = geometry_msgs.msg.Pose()
        quat_tf = quaternion_from_euler(roll, pitch, yaw)
        orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
        gripper_pose.position.x = x
        gripper_pose.position.y = y
        gripper_pose.position.z = z
        gripper_pose.orientation = orientation
        
        success = self.move_to_pose_msg(gripper_pose)

        if success:
            status = True
        else:
            status = False

        return status

    def move_to_target(self, target):
        # moves to stored target
        status = False

        self.arm.set_named_target(target)

        error_code_val, plan, planning_time, error_code = self.arm.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        error_code_val = self.arm.execute(plan)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        status = True
        return status

    def move_gripper(self, gripper_width):
        status = False

        pos = np.clip(0.51018 * gripper_width + 0.0134679, 0.021, 0.057)
        joint_goal = self.gripper.get_current_joint_values()
        joint_goal[0] = pos
        joint_goal[1] = -pos

        error_code_val, plan, planning_time, error_code = self.gripper.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        error_code_val = self.gripper.execute(plan)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        status = True
        return status

    def move_gripper_to_target(self, target):
        # moves to stored target

        status = False

        self.gripper.set_named_target(target)

        error_code_val, plan, planning_time, error_code = self.gripper.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        error_code_val = self.gripper.execute(plan)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)

        if success:
            pass
        else:
            status = False
            return status

        status = True
        return status

    def explore_environment(self):
        # Runs a routine to Map environment 
        status = False

        self.reset()

        for target in ("step6","step1", "step2", "step3", "step4", "step5", "step6"):
            success = self.move_to_target(target)
            if success:
                pass
            else:
                status = False
                return status
        
        status = True
        return status
  
    def pick(self, object_id, retreat=None):
        status = False

        self.enable_camera(False) # turns off Object Adder

        grasps = self.get_grasps(object_id, retreat=retreat) # Gets grasp from grasp planner
        
        for grasp in grasps:
            success = self.arm.pick(object_id, grasp, plan_only=True)
            if success > 0:
                self.arm.pick(object_id, grasp)
                status = True
                break
            else:
                status = False

        if status:
            self.attached_object = object_id
            pass
        else:
            self.enable_camera(True)
            
        return status

    def drop(self, pose):
        status = False

        success = self.move_to_pose_msg(pose)

        if success:
            pass
        else:
            status = False
            return status

        success = self.move_gripper_to_target('Open')

        if success:
            pass
        else:
            status = False
            return status

        self.detach_object(object_id)
        self.remove_object(object_id)
        self.enable_camera(True)
        
        status = True
        return status

    def place(self, pose):
        status = False

        if self.attached_object == None or \
        len(self.scene.get_attached_objects([self.attached_object])) == 0:
            rospy.logerr("No object is attached")
            status = False
            return status

        success = self.arm.place(self.attached_object, pose)

        if success > 0:
            status = True
        else:
            status = False
            return status

        self.remove_object(self.attached_object)
        self.enable_camera(True)

        return status

    def reset(self):
        status = False

        try:
            self.enable_camera(True)
            self.detach_and_remove_all_objects()
            self.wait(2)
            self.clear_octomap()
            status = True
        except:
            status = False

        return status        

    def wait(self, sec):
        rate = rospy.Rate(1/sec)
        rate.sleep()

    def get_grasps(self,object_id, retreat=None):
        rospy.wait_for_service('grasp_planning_service')
        req = GraspsRequest()
        req.object_id = object_id
        req.retreat = retreat
        resp = self.grasp_planning_service(req)
        grasps = resp.grasps

        print(grasps)
        

        return grasps

    def enable_camera(self, on):
        rospy.wait_for_service('/mux_image_rect/select')
        rospy.wait_for_service('/mux_pointcloud/select')

        try:
            select_image_rect_service = rospy.ServiceProxy('/mux_image_rect/select', MuxSelect)
            select_pointcloud_service = rospy.ServiceProxy('/mux_pointcloud/select', MuxSelect)

            req_pointcloud = MuxSelectRequest()
            req_image_rect = MuxSelectRequest()
            if on:  
                req_pointcloud.topic = '/camera/depth/color/points'
                req_image_rect.topic = '/camera/color/image_rect'
            else:
                req_pointcloud.topic = 'none'
                req_image_rect.topic = 'none'

            select_pointcloud_service(req_pointcloud)
            select_image_rect_service(req_image_rect)

        except rospy.ServiceException as e:         
            print("Enable camera service calls failed: %s"%e)

    def detach_and_remove_all_objects(self):
        self.wait(0.1)
        self.scene.remove_attached_object(link='vx300s/ee_gripper_link')
        self.wait(0.1)
        self.scene.remove_world_object()
        self.wait(0.1)

    def remove_object(self, object_id):
        self.wait(0.1)
        self.scene.remove_world_object(object_id)
        self.wait(0.1)

    def detach_object(self, object_id):
        self.wait(0.1)
        self.scene.remove_attached_object(link='vx300s/ee_gripper_link', name=object_id)
        self.attached_object = None
        self.wait(0.1)

    def clear_octomap(self):
        rospy.wait_for_service('/vx300s/clear_octomap')

        try:
            clear_octomap_service = rospy.ServiceProxy('/vx300s/clear_octomap', Empty)
            clear_octomap_service()

        except rospy.ServiceException as e:         
            print("Clear octomap service call failed: %s"%e)

    def __open_gripper_posture(self):
        posture = trajectory_msgs.msg.JointTrajectory()

        posture.joint_names = ["left_finger", "right_finger"]

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0.057, -0.057]
        point.time_from_start = rospy.Duration(0.5)

        posture.points.append(point)

        return posture