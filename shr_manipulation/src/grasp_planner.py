#! /usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_msgs.msg
import trajectory_msgs.msg
import moveit_commander
import tf2_geometry_msgs
from std_srvs.srv import Empty
from shr_interfaces.srv import Grasps, GraspsResponse
from tf.transformations import quaternion_from_euler
from numpy import linspace
import math
import sys
import yaml
import numpy as np

tau = 2 * math.pi 

stream = open(rospy.get_param("/object_db"), 'r')
objects = yaml.safe_load(stream)

class GraspPlanner():
    def __init__(self):
        rospy.init_node('grasp_planner')

        moveit_commander.roscpp_initialize(sys.argv)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()
        self.arm = moveit_commander.MoveGroupCommander("interbotix_arm")

        self.pick_grasps_service = rospy.Service('grasp_planning_service', Grasps, self.pick_grasps)

        self.finger_max_in = .025
        self.finger_max_out = .0285
        self.max_gripper_width = .08


    def pick_grasps(self, req):
        grasps = []
        object = self.scene.get_objects([req.object_id])[req.object_id]
        primitive = object.primitives[0]

        if primitive.type == primitive.BOX:
            grasps = self.box_grasps(
                object, 
                retreat=req.retreat, 
                axis_constraints=objects[req.object_id]['axis_constraints']
            )
        elif primitive.type == primitive.CYLINDER:
            grasps = self.cylinder_grasps(
                object, 
                retreat=req.retreat, 
                axis_constraints=objects[req.object_id]['axis_constraints']
            )

        grasps.sort(key=lambda x: x.grasp_quality, reverse=True)

        resp = GraspsResponse()
        resp.grasps = grasps
        return resp

    def cylinder_grasps(self, object, retreat=None, axis_constraints=[]):
        grasps = []
        dim = object.primitives[0].dimensions
        c_h = dim[0]
        c_r = dim[1]
        fi = self.finger_max_in
        fo = self.finger_max_out

        min_dist = .005
        finger_range_num = 3
        height_range_num = 5
        circle_range_num = 12
        pitch_range_num = 3
        pitch_range = tau/12


        # grabbing side
        if 's' not in axis_constraints and c_r * 2 < self.max_gripper_width and fi + fo > c_r:
            finger_max_in = c_r-fi if fi+fo < 2*c_r else max(c_r-fi, fo-c_r) # todo
            for h in linspace(-c_h/2, c_h/2, height_range_num):
                for fr in linspace(finger_max_in, fo-min_dist, finger_range_num):
                    for roll in (0, tau/2):
                        for pitch in linspace(-pitch_range/2, pitch_range/2, pitch_range_num):
                            for yaw in linspace(0, tau - tau/circle_range_num, circle_range_num):
                                y = - fr * math.sin(yaw) 
                                x = - fr * math.cos(yaw)

                                gripper_pose = geometry_msgs.msg.PoseStamped()
                                quat_tf = quaternion_from_euler(roll, pitch, yaw)
                                orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                                gripper_pose.pose.position.x = x
                                gripper_pose.pose.position.y = y
                                gripper_pose.pose.position.z = h
                                gripper_pose.pose.orientation = orientation

                                score = 0
                                score -= (h) ** 2
                                score -= (fr - finger_max_in) ** 2
                                score -= (pitch)**2

                                gripper_world_pose = self.get_gripper_world_pose(gripper_pose, object.pose)
                                grasp_msg = self.grasp_msg(object, gripper_world_pose, 2*c_r, retreat, score)
                                
                                grasps.append(grasp_msg)

        #grabbing top and bottom
        if c_r * 2 < self.max_gripper_width:
            if '+z' not in axis_constraints:
                finger_max_in = c_h/2-fi if fi+fo < c_h else c_h/2+fo-c_h
                for fr in linspace(finger_max_in, (c_h/2)+fo-min_dist, finger_range_num):
                    for roll in linspace(0, tau - tau/circle_range_num, circle_range_num):
                                gripper_pose = geometry_msgs.msg.PoseStamped()
                                quat_tf = quaternion_from_euler(roll, tau/4, 0)
                                orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                                gripper_pose.pose.position.x = 0
                                gripper_pose.pose.position.y = 0
                                gripper_pose.pose.position.z = fr
                                gripper_pose.pose.orientation = orientation

                                score = 0
                                score -= (fr - ((c_h/2)-fi)) ** 2

                                gripper_world_pose = self.get_gripper_world_pose(gripper_pose, object.pose)
                                grasp_msg = self.grasp_msg(object, gripper_world_pose, 2*c_r, retreat, score)
                                
                                grasps.append(grasp_msg)

            if '-z' not in axis_constraints:
                finger_max_in = -c_h/2+fi if fi+fo < c_h else -c_h/2-fo+c_h
                for fr in linspace(finger_max_in, -(c_h/2)-fo+min_dist, finger_range_num):
                    for roll in linspace(0, tau - tau/circle_range_num, circle_range_num):
                                gripper_pose = geometry_msgs.msg.PoseStamped()
                                quat_tf = quaternion_from_euler(roll, -tau/4, 0)
                                orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                                gripper_pose.pose.position.x = 0
                                gripper_pose.pose.position.y = 0
                                gripper_pose.pose.position.z = fr
                                gripper_pose.pose.orientation = orientation

                                score = 0
                                score -= (fr - ((c_h/2)-fi)) ** 2

                                gripper_world_pose = self.get_gripper_world_pose(gripper_pose, object.pose)
                                grasp_msg = self.grasp_msg(object, gripper_world_pose, 2*c_r, retreat, score)
                                
                                grasps.append(grasp_msg)

        # grabbing side, gripper other way
        
        if 's' not in axis_constraints and c_h < self.max_gripper_width:
            finger_max_in = c_r-fi if fi+fo < 2*c_r else fo-c_r # todo
            for fr in linspace(finger_max_in, c_r+fo-min_dist, finger_range_num):
                for roll in (tau/4, -tau/4):
                    for yaw in linspace(0, tau - tau/circle_range_num, circle_range_num):
                        y = - fr * math.sin(yaw) 
                        x = - fr * math.cos(yaw)

                        gripper_pose = geometry_msgs.msg.PoseStamped()
                        quat_tf = quaternion_from_euler(roll, 0, yaw)
                        orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                        gripper_pose.pose.position.x = x
                        gripper_pose.pose.position.y = y
                        gripper_pose.pose.position.z = 0
                        gripper_pose.pose.orientation = orientation

                        score = 0
                        score -= (fr - finger_max_in) ** 2

                        gripper_world_pose = self.get_gripper_world_pose(gripper_pose, object.pose)
                        grasp_msg = self.grasp_msg(object, gripper_world_pose, c_h, retreat, score)
                        
                        grasps.append(grasp_msg)

        return grasps
   
    def box_grasps(self, object, retreat=None, axis_constraints=[]):
        grasps = []
        dim = object.primitives[0].dimensions
        box_x = dim[0]
        box_y = dim[1]
        box_z = dim[2]
        fi = self.finger_max_in
        fo = self.finger_max_out

        min_dist = .01
        finger_range_num = 3
        box_range_num = 5
        rotation_range_num = 3
        rotation_range = tau/20

        grasps_info = [
            { # -y 1
                'face': '-y',
                'rotation': (0,0,tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 2,
                'box_width_axis_index': 0
            },
            { # -y 2
                'face': '-y',
                'rotation': (tau/2,0,tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 2,
                'box_width_axis_index': 0
            },
            { # -y 3
                'face': '-y',
                'rotation': (tau/4,0,tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 0,
                'box_width_axis_index': 2
            },
            { # -y 4
                'face': '-y',
                'rotation': (-tau/4,0,tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 0,
                'box_width_axis_index': 2
            },

            { # +y 1
                'face': '+y',
                'rotation': (0,0,-tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 2,
                'box_width_axis_index': 0
            },
            { # +y 2
                'face': '+y',
                'rotation': (tau/2,0,-tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 2,
                'box_width_axis_index': 0
            },
            { # +y 3
                'face': '+y',
                'rotation': (tau/4,0,-tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 0,
                'box_width_axis_index': 2
            },
            { # +y 4
                'face': '+y',
                'rotation': (-tau/4,0,-tau/4),
                'finger_range_axis_index': 1,
                'box_range_axis_index': 0,
                'box_width_axis_index': 2
            },

            { # +x 1
                'face': '+x',
                'rotation': (0,0,tau/2),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 2,
                'box_width_axis_index': 1
            },
            { # +x 2
                'face': '+x',
                'rotation': (tau/2,0,tau/2),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 2,
                'box_width_axis_index': 1
            },
            { # +x 3
                'face': '+x',
                'rotation': (tau/4,0,tau/2),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 1,
                'box_width_axis_index': 2
            },
            { # +x 4
                'face': '+x',
                'rotation': (-tau/4,0,tau/2),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 1,
                'box_width_axis_index': 2
            },

            { # -z 1
                'face': '-z',
                'rotation': (0,-tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 0,
                'box_width_axis_index': 1                
            },
            { # -z 2
                'face': '-z',
                'rotation': (tau/2,-tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 0,
                'box_width_axis_index': 1    
            },
            { # -z 3
                'face': '-z',
                'rotation': (tau/4,-tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 1,
                'box_width_axis_index': 0    
            },
            { # -z 4
                'face': '-z',
                'rotation': (-tau/4,-tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 1,
                'box_width_axis_index': 0    
            },

            { # -x 1
                'face': '-x',
                'rotation': (0,0,0),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 2,
                'box_width_axis_index': 1    
            },
            { # -x 2
                'face': '-x',
                'rotation': (tau/2,0,0),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 2,
                'box_width_axis_index': 1    
            },
            { # -x 3
                'face': '-x',
                'rotation': (tau/4,0,0),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 1,
                'box_width_axis_index': 2    
            },
            { # -x 4
                'face': '-x',
                'rotation': (-tau/4,0,0),
                'finger_range_axis_index': 0,
                'box_range_axis_index': 1,
                'box_width_axis_index': 2  
            },

            { # +z 1
                'face': '+z',
                'rotation': (0,tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 0,
                'box_width_axis_index': 1  
            },
            { # +z 2
                'face': '+z',
                'rotation': (tau/2,tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 0,
                'box_width_axis_index': 1  
            },
            { # +z 3
                'face': '+z',
                'rotation': (tau/4,tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 1,
                'box_width_axis_index': 0  
            },
            { # +z 4
                'face': '+z',
                'rotation': (-tau/4,tau/4,0),
                'finger_range_axis_index': 2,
                'box_range_axis_index': 1,
                'box_width_axis_index': 0  
            },
        ]

        finger_ranges = {
            '-y': (-(box_y/2)+fi, -(box_y/2)-fo+min_dist),
            '+y': ((box_y/2)-fi, (box_y/2)+fo-min_dist),
            '+x': ((box_x/2)-fi, (box_x/2)+fo-min_dist),
            '-z': (-(box_z/2)+fi, -(box_z/2)-fo+min_dist),
            '-x': (-(box_x/2)+fi, -(box_x/2)-fo+min_dist),
            '+z': ((box_z/2)-fi, (box_z/2)+fo-min_dist),
        }

        small_finger_ranges = {
            '-y': (-(box_y/2)-fo+box_y, -(box_y/2)-fo+min_dist),
            '+y': ((box_y/2)+fo-box_y, (box_y/2)+fo-min_dist),
            '+x': ((box_x/2)+fo-box_x, (box_x/2)+fo-min_dist),
            '-z': (-(box_z/2)-fo+box_z, -(box_z/2)-fo+min_dist),
            '-x': (-(box_x/2)-fo+box_x, -(box_x/2)-fo+min_dist),
            '+z': ((box_z/2)+fo-box_z, (box_z/2)+fo-min_dist),
        }

        for g in grasps_info:
            box_w = dim[g['box_width_axis_index']]
            box_f = dim[g['finger_range_axis_index']]
            box_l = dim[g['box_range_axis_index']]

            if g['face'] in axis_constraints or box_w  > self.max_gripper_width:
                continue

            if fi + fo < box_f:
                finger_range = finger_ranges[g['face']]
            else:
                finger_range = small_finger_ranges[g['face']]


            for i in linspace(finger_range[0], finger_range[1], finger_range_num):
                for j in linspace(-box_l/2, box_l/2, box_range_num):
                    for k in linspace(rotation_range/2, -rotation_range/2, rotation_range_num):
                        gripper_pose = geometry_msgs.msg.PoseStamped()
                        pos = [0,0,0]
                        pos[g['finger_range_axis_index']] = i
                        pos[g['box_range_axis_index']] = j
                        e = g['rotation']
                        quat_tf = quaternion_from_euler(e[0], e[1] + k, e[2])
                        orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
                        gripper_pose.pose.position.x = pos[0]
                        gripper_pose.pose.position.y = pos[1]
                        gripper_pose.pose.position.z = pos[2]
                        gripper_pose.pose.orientation = orientation

                        score = 0
                        score -= (j) ** 2
                        score -= (i - finger_range[0]) ** 2
                        score -= (k)**2

                        gripper_world_pose = self.get_gripper_world_pose(gripper_pose, object.pose)
                        grasp_msg = self.grasp_msg(object, gripper_world_pose, box_w, retreat, score)
                        
                        grasps.append(grasp_msg)
        
        return grasps

    def open_gripper_posture(self):
        posture = trajectory_msgs.msg.JointTrajectory()

        posture.joint_names = ["left_finger", "right_finger"]

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        point.positions = [0.057, -0.057]
        point.time_from_start = rospy.Duration(0.5)

        posture.points.append(point)

        return posture

    def close_gripper_posture(self, gripper_width):
        posture = trajectory_msgs.msg.JointTrajectory()

        posture.joint_names = ["left_finger", "right_finger"]

        point = trajectory_msgs.msg.JointTrajectoryPoint()
        pos = np.clip(0.51018 * gripper_width + 0.0134679, 0.021, 0.057)
        point.positions =[pos, -pos] #[0.036, -0.036]
        point.time_from_start = rospy.Duration(0.5)

        posture.points.append(point)

        return posture
    
    def grasp_msg(self, object, gripper_pose, gripper_width, retreat, score):
        grasp = moveit_msgs.msg.Grasp()
        
        grasp.grasp_pose.header.frame_id = "vx300s/base_link"
        grasp.grasp_pose.pose = gripper_pose.pose

        grasp.pre_grasp_approach.direction.header.frame_id = "vx300s/ee_gripper_link"
        grasp.pre_grasp_approach.direction.vector.x = 0.5
        grasp.pre_grasp_approach.min_distance = self.finger_max_in + self.finger_max_out + 0.06
        grasp.pre_grasp_approach.desired_distance = 0.15 + self.finger_max_in + self.finger_max_out

        retreat_vectors = {
            '+x':[.5,0,0],
            '-x':[-.5,0,0],
            '+y':[0,.5,0],
            '-y':[0,-.5,0],
            '+z':[0,0,.5],
            '-z':[0,0,-.5],
        }

        grasp.post_grasp_retreat.direction.header.frame_id = "vx300s/base_link"

        if retreat != '' and (retreat in retreat_vectors):
            vector = retreat_vectors[retreat]
            grasp.post_grasp_retreat.direction.vector.x = vector[0]
            grasp.post_grasp_retreat.direction.vector.y = vector[1]
            grasp.post_grasp_retreat.direction.vector.z = vector[2]
        else:
            retreat_vector = geometry_msgs.msg.Vector3Stamped()
            v = objects[object.id]['retreat_vector']
            retreat_vector.vector.x = v[0]
            retreat_vector.vector.y = v[1]
            retreat_vector.vector.z = v[2]

            retreat_vector = self.get_vector_in_world(retreat_vector, object.pose)
            
            grasp.post_grasp_retreat.direction.vector = retreat_vector.vector



        grasp.post_grasp_retreat.min_distance = 0.01
        grasp.post_grasp_retreat.desired_distance = 0.25

        grasp.pre_grasp_posture = self.open_gripper_posture()
        grasp.grasp_posture = self.close_gripper_posture(gripper_width)

        grasp.grasp_quality = score

        return grasp

    def get_gripper_world_pose(self, gripper_pose, object_pose):
        trans = geometry_msgs.msg.TransformStamped()
        trans.transform.translation.x = object_pose.position.x
        trans.transform.translation.y = object_pose.position.y
        trans.transform.translation.z = object_pose.position.z
        trans.transform.rotation = object_pose.orientation
        gripper_world_pose = tf2_geometry_msgs.do_transform_pose(gripper_pose, trans)

        return gripper_world_pose

    def get_vector_in_world(self, vector, object_pose):
        trans = geometry_msgs.msg.TransformStamped()
        trans.transform.translation.x = object_pose.position.x
        trans.transform.translation.y = object_pose.position.y
        trans.transform.translation.z = object_pose.position.z
        trans.transform.rotation = object_pose.orientation
        vector_in_world = tf2_geometry_msgs.do_transform_vector3(vector, trans)

        return vector_in_world


if __name__ == '__main__':
    grasp_planner = GraspPlanner()
    rospy.spin()