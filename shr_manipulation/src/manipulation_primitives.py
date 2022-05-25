#! /usr/bin/env python3

import rospy
import moveit_commander
import numpy as np
import yaml

from tf.transformations import quaternion_from_euler
import math

import moveit_msgs.msg
import geometry_msgs.msg

from shr_interfaces.srv import Grasps, GraspsRequest
from std_srvs.srv import Empty
from topic_tools.srv import MuxSelect, MuxSelectRequest


tau = 2 * math.pi 
stream = open(rospy.get_param("/object_db"), 'r')
objects = yaml.safe_load(stream)


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
        self.world_frame = "world"

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

    def move_gripper(self, gripper_width):  # todo must fix
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
  
    def pick(self, object_id, retreat=''):
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
            pass
        else:
            self.enable_camera(True)
            
        return status

    def drop(self, object_id, pose):
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

    def place(self, object_id, pose):
        status = False

        if len(self.scene.get_attached_objects([object_id])) == 0:
            rospy.logerr("No object is attached")
            status = False
            return status

        success = self.arm.place(object_id, pose)

        if success > 0:
            status = True
        else:
            status = False
            return status

        self.enable_camera(True)

        return status

    def reset(self):
        status = False

        try:
            self.enable_camera(True)
            self.wait(0.1)
            self.scene.remove_attached_object(link='vx300s/ee_gripper_link')
            self.wait(0.1)
            self.scene.remove_world_object()
            self.wait(1)
            self.clear_octomap()
            status = True
        except Exception as e:
            rospy.logerr('Reset failed: %s' % e)
            status = False

        return status        

    def wait(self, sec):
        status = False

        try:
            rate = rospy.Rate(1/sec)
            rate.sleep()
            status = True
        except:
            status = False

        return status 
        
    def get_grasps(self,object_id, retreat=''):
        rospy.wait_for_service('grasp_planning_service')
        req = GraspsRequest()
        req.object_id = object_id
        req.retreat = retreat
        resp = self.grasp_planning_service(req)
        grasps = resp.grasps        

        return grasps

    def enable_camera(self, on):
        status = False  

        try:
            rospy.wait_for_service('/mux_image_rect/select')
            rospy.wait_for_service('/mux_pointcloud/select')

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

            status = True

        except rospy.ServiceException as e:         
            rospy.logerr("Enable camera service calls failed: %s"%e)
            status = False
        
        return status    

    def remove_object(self, object_id):
        status = False

        try:
            self.wait(0.1)
            self.scene.remove_world_object(object_id)
            self.wait(0.1)
            status = True
        except:
            status = False

        return status        

    def detach_object(self, object_id):

        status = False

        try:
            self.wait(0.1)
            self.scene.remove_attached_object(link='vx300s/ee_gripper_link', name=object_id)
            self.wait(0.1)
            status = True
        except:
            status = False

        return status 

    def clear_octomap(self):
        status = False

        try:
            rospy.wait_for_service('/vx300s/clear_octomap')
            clear_octomap_service = rospy.ServiceProxy('/vx300s/clear_octomap', Empty)
            clear_octomap_service()
            status = True

        except rospy.ServiceException as e:         
            rospy.logerr("Clear octomap service call failed: %s"%e)
            status = False
        
        return status

    def add_object(self, object_id, pose):
        status = False

        pose_stamped = geometry_msgs.msg.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = self.world_frame
        
        dimensions = objects[object_id]['dimensions']
        type = objects[object_id]['type']

        try:
            if type == 'box':
                if len(dimensions) != 3:
                    raise Exception("box dimensions wrong")

                self.wait(0.1)
                self.scene.add_box(object_id, pose_stamped, size=dimensions)
                self.wait(0.1)
            elif type == 'cylinder':
                if len(dimensions) != 2:
                    raise Exception("cylinder dimensions wrong")

                self.wait(0.1)
                h, r = dimensions
                self.scene.add_cylinder(object_id, pose_stamped, h, r)
                self.wait(0.1)
            else:
                raise Exception("Invalid type")
            
            status = True
            return status

        except Exception as e:
            rospy.logerr(e)
            status = False
            return status     