#! /usr/bin/env python3

import rospy
import actionlib
from actionlib_msgs.msg import GoalStatus
from shape_msgs.msg import SolidPrimitive
import geometry_msgs.msg
from geometry_msgs.msg import Pose, Quaternion, TransformStamped
from tf.transformations import quaternion_from_euler
import tf2_geometry_msgs
import tf2_ros
from scipy.spatial.transform import Rotation


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


import yaml

class ManipulationClient():
    def __init__(self, ns):  
        stream = open(rospy.get_param("/object_db"), 'r')
        self.objects = yaml.safe_load(stream)   
        self.ns = ns

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

    def flatten_pose(self, pose):
        q = pose.orientation
        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])
        euler = rot.as_euler('xyz', degrees=False)
        quat_tf = Rotation.from_euler('xyz', [0,0,euler[2]], degrees=False).as_quat()
        pose.orientation =  geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

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
        rospy.loginfo(scene_objects)
        status = True
        return scene_objects, status

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

