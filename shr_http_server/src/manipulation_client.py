#! /usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus

from std_srvs.srv import SetBool, SetBoolRequest, Empty, EmptyRequest, Trigger, TriggerRequest
from shr_interfaces.srv import String, StringRequest, Float, FloatRequest, AddObject, AddObjectRequest

from shr_interfaces.msg import \
    DropAction, DropGoal, DropFeedback, DropResult, \
    MoveGripperAction, MoveGripperGoal, MoveGripperFeedback, MoveGripperResult, \
    MoveToPoseAction, MoveToPoseGoal, MoveToPoseFeedback, MoveToPoseResult, \
    MoveToTargetAction, MoveToTargetGoal, MoveToTargetFeedback, MoveToTargetResult, \
    PickAction, PickGoal, PickFeedback, PickResult, \
    PlaceAction, PlaceGoal, PlaceFeedback, PlaceResult

class ManipulationClient():
    def __init__(self, ns):     
        self.ns = ns
        # self.move_to_pose_as = actionlib.SimpleActionServer(
        #     'move_to_pose', 
        #     MoveToPoseAction, 
        #     execute_cb=self.move_to_pose_cb, 
        #     auto_start=False
        # )
        # self.move_to_pose_as.start()
        # self.move_to_target_as = actionlib.SimpleActionServer(
        #     'move_to_target', 
        #     MoveToTargetAction, 
        #     execute_cb=self.move_to_target_cb, 
        #     auto_start=False
        # )
        # self.move_to_target_as.start()
        # self.move_gripper_as = actionlib.SimpleActionServer(
        #     'move_gripper', 
        #     MoveGripperAction, 
        #     execute_cb=self.move_gripper_cb, 
        #     auto_start=False
        # )
        # self.move_gripper_as.start()
        # self.move_gripper_to_target_as = actionlib.SimpleActionServer(
        #     'move_gripper_to_target', 
        #     MoveToTargetAction, 
        #     execute_cb=self.move_gripper_to_target_cb, 
        #     auto_start=False
        # )
        # self.move_gripper_to_target_as.start()
        # self.pick_as = actionlib.SimpleActionServer(
        #     'pick', 
        #     PickAction, 
        #     execute_cb=self.pick_cb, 
        #     auto_start=False
        # )
        # self.pick_as.start()
        # self.place_as = actionlib.SimpleActionServer(
        #     'place_object', 
        #     PlaceAction, 
        #     execute_cb=self.place_cb, 
        #     auto_start=False
        # )
        # self.place_as.start()
        # self.drop_as = actionlib.SimpleActionServer(
        #     'drop', 
        #     DropAction, 
        #     execute_cb=self.drop_cb, 
        #     auto_start=False
        # )
        # self.drop_as.start()

        # self.clear_octomap_service = rospy.Service("clear_octomap_node", Trigger, self.clear_octomap_cb)

        # self.remove_object_service = rospy.Service("remove_object", String, self.remove_object_cb)

        # self.detach_object_service = rospy.Service("detach_object", String, self.detach_object_cb)

        # self.reset_service = rospy.Service("reset", Empty, self.reset_cb)

        # self.wait_service = rospy.Service("wait", Float, self.wait_cb)

        # self.enable_camera_service = rospy.Service("enable_camera", SetBool, self.enable_camera_cb)

    def move_to_pose_client(self, pose):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/move_to_pose', MoveToPoseAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact move_to_pose action server");
            status = False
            return status

        goal = MoveToPoseGoal()
        goal.pose = pose
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("MoveToPose failed")
            status = False
            return status

        rospy.loginfo("MoveToPose succeeded")
        status = True
        return status

    def move_to_target_client(self, target):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/move_to_target', MoveToTargetAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact move_to_target action server");
            status = False
            return status

        goal = MoveToTargetGoal()
        goal.target = target
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("MoveToTarget failed")
            status = False
            return status

        rospy.loginfo("MoveToTarget succeeded")
        status = True
        return status

    def move_gripper_to_target_client(self, target):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/move_gripper_to_target', MoveToTargetAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact move_gripper_to_target action server");
            status = False
            return status

        goal = MoveToTargetGoal()
        goal.target=target
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("MoveGripperToTarget failed")
            status = False
            return status

        rospy.loginfo("MoveGripperToTarget succeeded")
        status = True
        return status

    def gripper_width_client(self, gripper_width):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/move_gripper', MoveGripperAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact move_gripper action server");
            status = False
            return status

        goal = MoveGripperGoal()
        goal.gripper_width = gripper_width
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("MoveGripper failed")
            status = False
            return status

        rospy.loginfo("MoveGripper succeeded")
        status = True
        return status

    def pick_client(self, object_id, retreat=''):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/pick', PickAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact pick action server");
            status = False
            return status

        goal = PickGoal()
        goal.object_id = object_id
        goal.retreat = retreat
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("Pick failed")
            status = False
            return status

        rospy.loginfo("Pick succeeded")
        status = True
        return status

    def place_client(self, object_id, pose):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/place', PlaceAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact place action server");
            status = False
            return status

        goal = PlaceGoal()
        goal.object_id = object_id
        goal.pose = pose
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("Place failed")
            status = False
            return status

        rospy.loginfo("Place succeeded")
        status = True
        return status

    def add_object_client(self, type, object_id, pose, dimensions):
        status = False
        try:
            rospy.wait_for_service(f'{self.ns}/add_object', timeout=rospy.Duration(2.0))
        except:
            rospy.logerr("Can't contact add_object service")
            status = False
            return status

        try:
            add_object = rospy.ServiceProxy(f'{self.ns}/add_object', AddObject)
            req = AddObjectRequest()
            req.type = type
            req.object_id = object_id
            req.pose = pose
            req.dimensions = dimensions
            res = add_object(req)
        except rospy.ServiceException as e:
            rospy.logerr("add_object service call failed: %s"%e)
            status = False
            return status

        if not res.success:
            rospy.logerr("AddObject failed")
            status = False
            return status

        rospy.loginfo("AddObject succeeded")
        status = True
        return status

    def remove_object_client(self, object_id):
        status = False

        try:
            rospy.wait_for_service(f'{self.ns}/remove_object', timeout=rospy.Duration(2.0))
        except:
            rospy.logerr("Can't contact remove_object service")
            status = False
            return status

        try:
            remove_object = rospy.ServiceProxy(f'{self.ns}/remove_object', String)
            req = StringRequest()
            req.data = object_id
            res = remove_object(req)
        except rospy.ServiceException as e:
            rospy.logerr("remove_object service call failed: %s"%e)
            status = False
            return status

        if not res.success:
            rospy.logerr("RemoveObject failed")
            status = False
            return status

        rospy.loginfo("RemoveObject succeeded")
        status = True
        return status