#! /usr/bin/env python3

from numpy import rot90
import rospy
import tf2_ros
import geometry_msgs.msg
import tf2_geometry_msgs
import moveit_commander
import yaml
from scipy.spatial.transform import Rotation

stream = open(rospy.get_param("/object_db"), 'r')
objects = yaml.safe_load(stream)

class ObjectAdder():
    def __init__(self):
        rospy.init_node('object_adder')
        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer)
        self.scene = moveit_commander.PlanningSceneInterface()
        self.world_frame = "world"

    def run(self, timeout=4):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            for id in objects:
                object = objects[id]

                try:
                    trans = self.tfBuffer.lookup_transform(object['frame_id'], self.world_frame, rospy.Time())
                except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                    continue

                pose = geometry_msgs.msg.Pose()
                pose.position.x = object['transform'][0]
                pose.position.y = object['transform'][1]
                pose.position.z = object['transform'][2]
                pose.orientation.x = object['transform'][3]
                pose.orientation.y = object['transform'][4]
                pose.orientation.z = object['transform'][5]
                pose.orientation.w = object['transform'][6]

                pose = self.transform_pose(pose, object['frame_id'], 'world')
                pose = self.flatten_pose(pose)

                pose.position.z = 0.0235 + object['dimensions'][2] / 2

                pose_stamped = geometry_msgs.msg.PoseStamped()
                pose_stamped.header.frame_id = 'world'
                pose_stamped.pose = pose
                
                if object['type'] == 'box':
                    self.scene.add_box(id, pose_stamped, size=object['dimensions'])
                else:
                    h, r = object['dimensions']
                    self.scene.add_cylinder(id, pose_stamped, h, r)

            rate.sleep()

    def transform_pose(self, pose, from_frame, to_frame):
        pose_stamped = tf2_geometry_msgs.PoseStamped()
        pose_stamped.pose = pose
        pose_stamped.header.frame_id = from_frame

        print(from_frame, to_frame)

        try:
            # ** It is important to wait for the listener to start listening. Hence the rospy.Duration(1)
            output_pose_stamped = self.tfBuffer.transform(pose_stamped, to_frame)
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            print('oh no')

        return output_pose_stamped.pose

    def flatten_pose(self,pose):
        q = pose.orientation


        rot = Rotation.from_quat([q.x, q.y, q.z, q.w])

        euler = rot.as_euler('xyz', degrees=False)


        quat_tf = Rotation.from_euler('xyz', [0,0,euler[2]], degrees=False).as_quat()

        pose.orientation =  geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])

        return pose





if __name__ == '__main__':
    object_adder = ObjectAdder()
    object_adder.run()
    moveit_commander.roscpp_shutdown()