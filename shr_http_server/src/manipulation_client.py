#! /usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
import geometry_msgs.msg
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs
import tf2_ros


from moveit_msgs.srv import GetPlanningScene, GetPlanningSceneRequest
from std_srvs.srv import SetBool, SetBoolRequest, Empty, EmptyRequest, Trigger, TriggerRequest
from shr_interfaces.srv import String, StringRequest, Float, FloatRequest, AddObject, AddObjectRequest, Transform, TransformRequest

from shr_interfaces.msg import \
    DropAction, DropGoal, DropFeedback, DropResult, \
    MoveGripperAction, MoveGripperGoal, MoveGripperFeedback, MoveGripperResult, \
    MoveToPoseAction, MoveToPoseGoal, MoveToPoseFeedback, MoveToPoseResult, \
    MoveToTargetAction, MoveToTargetGoal, MoveToTargetFeedback, MoveToTargetResult, \
    PickAction, PickGoal, PickFeedback, PickResult, \
    PlaceAction, PlaceGoal, PlaceFeedback, PlaceResult, \
    ExecuteBehaviorTreeAction, ExecuteBehaviorTreeGoal, ExecuteBehaviorTreeFeedback, ExecuteBehaviorTreeResult

class ManipulationClient():
    def __init__(self, ns):     
        self.ns = ns

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

    def transform_pose(self, pose, from_frame, to_frame):
        rospy.wait_for_service('transform_pose')
        try:
            transform_pose = rospy.ServiceProxy('transform_pose', Transform)
            req = TransformRequest()
            req.pose = pose
            req.from_frame = from_frame
            req.to_frame = to_frame
            pose = transform_pose(req).pose
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)

        return pose


    def get_scene_objects(self):
        status = False

        try:
            rospy.wait_for_service(f'{self.ns}/get_planning_scene', timeout=rospy.Duration(2.0))
        except:
            rospy.logerr("Can't contact get_planning_scene service")
            status = False
            return [], status

        try:
            get_planning_scene = rospy.ServiceProxy(f'{self.ns}/get_planning_scene', GetPlanningScene)
            res = get_planning_scene(GetPlanningSceneRequest())
        except rospy.ServiceException as e:
            rospy.logerr("get_planning_scene service call failed: %s"%e)
            status = False
            return [], status

        scene_objects = []
        for obj in res.scene.world.collision_objects:
            if len(obj.primitives) == 1 and obj.id == 'cube': # modify to send only cube
                obj_type = {
                    SolidPrimitive.BOX: 'box',
                    SolidPrimitive.CYLINDER: 'cylinder'
                }

                type = obj_type[obj.primitives[0].type]

                pose = self.transform_pose(obj.pose, "world", "ipad_camera")

                scene_objects.append(
                    {
                        'object_id': obj.id,
                        'posX' : pose.position.x,
                        'posY' : -pose.position.y,
                        'posZ' : pose.position.z,
                        'rotX' : pose.orientation.x,
                        'rotY' : pose.orientation.y,
                        'rotZ' : pose.orientation.z,
                        'rotW' : pose.orientation.w,
                        'boxX' : obj.primitives[0].dimensions[0],
                        'boxY' : obj.primitives[0].dimensions[1],
                        'boxZ' : obj.primitives[0].dimensions[2]
                    }
                )

        rospy.loginfo("get_scene_objects succeeded")
        status = True
        return scene_objects[0], status

    def execute_behavior_tree_client(self, xml):
        status = False

        client = actionlib.SimpleActionClient(f'{self.ns}/execute_behavior_tree', ExecuteBehaviorTreeAction)

        if not client.wait_for_server(rospy.Duration(2.0)):
            rospy.logerr("Can't contact execute_behavior_tree action server")
            status = False
            return status

        goal = ExecuteBehaviorTreeGoal()
        goal.xml = xml
        client.send_goal(goal)

        client.wait_for_result()

        if client.get_state() != GoalStatus.SUCCEEDED:
            rospy.logerr("ExecuteBehaviorTree failed")
            status = False
            return status

        rospy.loginfo("ExecuteBehaviorTree succeeded")
        status = True
        return status

