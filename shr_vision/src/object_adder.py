#! /usr/bin/env python3

import rospy
import tf2_ros
import geometry_msgs.msg
import moveit_commander
import yaml

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

                pose = geometry_msgs.msg.PoseStamped()
                pose.header.frame_id = object['frame_id']
                pose.pose.position.x = object['transform'][0]
                pose.pose.position.y = object['transform'][1]
                pose.pose.position.z = object['transform'][2]
                pose.pose.orientation.x = object['transform'][3]
                pose.pose.orientation.y = object['transform'][4]
                pose.pose.orientation.z = object['transform'][5]
                pose.pose.orientation.w = object['transform'][6]
                
                if object['type'] == 'box':
                    self.scene.add_box(id, pose, size=object['dimensions'])
                else:
                    h, r = object['dimensions']
                    self.scene.add_cylinder(id, pose, h, r)

            rate.sleep()




if __name__ == '__main__':
    object_adder = ObjectAdder()
    object_adder.run()
    moveit_commander.roscpp_shutdown()