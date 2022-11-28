#! /usr/bin/env python3

import rospy
import actionlib
import actionlib_msgs.msg
from manipulation_actions import ManipulationActions
import math

from shr_interfaces.msg import \
    FloatAction, FloatFeedback, FloatResult, \
    PoseAction, PoseFeedback, PoseResult, \
    StringAction, StringFeedback, StringResult \
    

class ManipulationActionServer():
    def __init__(self):
        super().__init__()

        move_groups = ['arm', 'gripper']
        self.ma = ManipulationActions(move_groups)

        self.move_to_pose_as = actionlib.SimpleActionServer(
            'move_to_pose', 
            PoseAction, 
            execute_cb=self.move_to_pose_cb, 
            auto_start=False
        )
        self.move_to_pose_as.start()

        self.move_to_target_as = actionlib.SimpleActionServer(
            'move_to_target', 
            StringAction, 
            execute_cb=self.move_to_target_cb, 
            auto_start=False
        )
        self.move_to_target_as.start()

        self.move_gripper_as = actionlib.SimpleActionServer(
            'move_gripper', 
            FloatAction, 
            execute_cb=self.move_gripper_cb, 
            auto_start=False
        )
        self.move_gripper_as.start()

        self.move_gripper_to_target_as = actionlib.SimpleActionServer(
            'move_gripper_to_target', 
            StringAction, 
            execute_cb=self.move_gripper_to_target_cb, 
            auto_start=False
        )
        self.move_gripper_to_target_as.start() 

        self.pregrasp_approach_as = actionlib.SimpleActionServer(
            'pregrasp_approach', 
            FloatAction, 
            execute_cb=self.pregrasp_approach_cb, 
            auto_start=False
        )
        self.pregrasp_approach_as.start()

        self.move_cartesian_path_as = actionlib.SimpleActionServer(
            'move_cartesian_path', 
            PoseAction, 
            execute_cb=self.move_cartesian_path_cb, 
            auto_start=False
        )
        self.move_cartesian_path_as.start()

    def move_to_pose_cb(self, goal):
        success = self.ma.move_to_pose('arm', goal.pose)
        if not success:
            self.move_to_pose_as.set_aborted()
            return
        self.ma.wait_for_result(self.move_to_pose_as)

    def move_to_target_cb(self, goal):
        success = self.ma.move_to_target('arm', goal.string)
        if not success:
            self.move_to_pose_as.set_aborted()
            return
        self.ma.wait_for_result(self.move_to_target_as)
         
    def move_gripper_cb(self, goal):
        success = self.ma.move_gripper('gripper', goal.value)
        if not success:
            self.move_to_pose_as.set_aborted()
            return
        self.ma.wait_for_result(self.move_gripper_as)

    def move_gripper_to_target_cb(self, goal):
        success = self.ma.move_to_target('gripper', goal.string)
        if not success:
            self.move_to_pose_as.set_aborted()
            return
        self.ma.wait_for_result(self.move_gripper_to_target_as)

    def pregrasp_approach_cb(self, goal):
        success = self.ma.pregrasp_approach('arm', goal.value)
        if not success:
            self.pregrasp_approach_as.set_aborted()
            return
        self.ma.wait_for_result(self.pregrasp_approach_as)

    def move_cartesian_path_cb(self, goal):
        success = self.ma.move_cartesian_path('arm', goal.pose)
        if not success:
            self.move_cartesian_path_as.set_aborted()
            return
        self.ma.wait_for_result(self.move_cartesian_path_as)

    # def pick_cb(self, goal):
    #     success = self.pick(goal.object_id, goal.retreat)

    #     if success:
    #         self.pick_as.set_succeeded()
    #     else:
    #         self.pick_as.set_aborted()

    # def place_cb(self, goal):
    #     success = self.place(goal.object_id, goal.pose)

    #     if success:
    #         self.place_as.set_succeeded()
    #     else:
    #         self.place_as.set_aborted()

    # def drop_cb(self, goal):
    #     success = self.drop(goal.object_id, goal.pose)

    #     if success:
    #         self.drop_as.set_succeeded()
    #     else:
    #         self.drop_as.set_aborted()

    # def clear_octomap_cb(self, req):
    #     res = TriggerResponse()
    #     res.success = self.clear_octomap()
    #     return res

    # def remove_object_cb(self, req):
    #     res = StringResponse()
    #     res.success = self.remove_object(req.data)
    #     return res

    # def detach_object_cb(self, req):
    #     res = StringResponse()
    #     res.success = self.detach_object(req.data)
    #     return res

    # def reset_cb(self, req):
    #     res = TriggerResponse()
    #     res.success = self.reset()
    #     return res

    # def wait_cb(self, req):
    #     res = FloatResponse()
    #     res.success = self.wait(req.data)
    #     return res
   
    # def enable_camera_cb(self,req):
    #     res = SetBoolResponse()
    #     res.success = self.enable_camera(req.data)
    #     return res

    # def add_object_cb(self, req):
    #     res = AddObjectResponse()
    #     res.success = self.add_object(req.object_id, req.pose)
    #     return res

if __name__ == "__main__":
    rospy.init_node('manipulation_action_server')
    manipulation_action_server = ManipulationActionServer()
    rospy.spin()