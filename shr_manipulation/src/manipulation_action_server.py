#! /usr/bin/env python3

import rospy
import actionlib
from manipulation_primitives import ManipulationPrimitives
import math

from std_srvs.srv import SetBool, SetBoolResponse, Empty, EmptyResponse

from shr_interfaces.msg import \
    DropAction, DropFeedback, DropResult, \
    MoveGripperAction, MoveGripperFeedback, MoveGripperResult, \
    MoveToPoseAction, MoveToPoseFeedback, MoveToPoseResult, \
    MoveToTargetAction, MoveToTargetFeedback, MoveToTargetResult, \
    PickAction, PickFeedback, PickResult, \
    PlaceAction, PlaceFeedback, PlaceResult

class ManipulationActionServer(ManipulationPrimitives):
    def __init__(self):
        super().__init__()
        
        self.move_to_pose_as = actionlib.SimpleActionServer(
            'move_to_pose', 
            MoveToPoseAction, 
            execute_cb=self.move_to_pose_cb, 
            auto_start=False
        )
        self.move_to_pose_as.start()
        self.move_to_target_as = actionlib.SimpleActionServer(
            'move_to_target', 
            MoveToTargetAction, 
            execute_cb=self.move_to_target_cb, 
            auto_start=False
        )
        self.move_to_target_as.start()
        self.move_gripper_as = actionlib.SimpleActionServer(
            'move_gripper', 
            MoveGripperAction, 
            execute_cb=self.move_gripper_cb, 
            auto_start=False
        )
        self.move_gripper_as.start()
        self.move_gripper_to_target_as = actionlib.SimpleActionServer(
            'move_gripper_to_target', 
            MoveToTargetAction, 
            execute_cb=self.move_gripper_to_target_cb, 
            auto_start=False
        )
        self.move_gripper_to_target_as.start()
        self.pick_as = actionlib.SimpleActionServer(
            'pick', 
            PickAction, 
            execute_cb=self.pick_cb, 
            auto_start=False
        )
        self.pick_as.start()
        self.place_as = actionlib.SimpleActionServer(
            'place_object', 
            PlaceAction, 
            execute_cb=self.place_cb, 
            auto_start=False
        )
        self.place_as.start()
        self.drop_as = actionlib.SimpleActionServer(
            'drop', 
            DropAction, 
            execute_cb=self.drop_cb, 
            auto_start=False
        )
        self.drop_as.start()

        self.reset_service = rospy.Service("reset", Empty, self.reset_cb)

        self.wait_service = rospy.Service("wait", Empty, self.wait_cb)

        self.enable_camera_service = rospy.Service("enable_camera", SetBool, self.enable_camera_cb)

    def move_to_pose_cb(self, goal):
        success = self.move_to_pose_msg(goal.pose)

        if success:
            self.move_to_pose_as.set_succeeded()
        else:
            self.move_to_pose_as.set_aborted()

    def move_to_target_cb(self, goal):
        success = self.move_to_target(goal.target)

        if success:
            self.move_to_target_as.set_succeeded()
        else:
            self.move_to_target_as.set_aborted()
         
    def move_gripper_cb(self, goal):
        success = self.move_gripper(goal.gripper_width)

        if success:
            self.move_gripper_as.set_succeeded()
        else:
            self.move_gripper_as.set_aborted()

    def move_gripper_to_target_cb(self, goal):
        success = self.move_gripper_to_target(goal.target)

        if success:
            self.move_gripper_to_target_as.set_succeeded()
        else:
            self.move_gripper_to_target_as.set_aborted()

    def pick_cb(self, goal):
        success = self.pick(goal.object_id, goal.retreat)

        if success:
            self.pick_as.set_succeeded()
        else:
            self.pick_as.set_aborted()

    def place_cb(self, goal):
        success = self.place(goal.pose)

        if success:
            self.place_as.set_succeeded()
        else:
            self.place_as.set_aborted()

    def drop_cb(self, goal):
        success = self.drop(goal.object_id, goal.pose)

        if success:
            self.drop_as.set_succeeded()
        else:
            self.drop_as.set_aborted()

    def reset_cb(self):
        self.reset()
        return EmptyResponse()

    def wait_cb(self):
        self.wait()
        return EmptyResponse()
   
    def enable_camera_cb(self,req):
        self.enable_camera(req.data)
        return SetBoolResponse()

if __name__ == "__main__":
    rospy.init_node('manipulation_action_server')
    manipulation_action_server = ManipulationActionServer()
    rospy.spin()