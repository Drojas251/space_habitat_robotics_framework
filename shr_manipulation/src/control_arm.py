#! /usr/bin/env python3

from requests import delete
import rospy
import moveit_commander
from manipulation_primitives import ManipulationPrimitives
import sys
import math
from tf.transformations import quaternion_from_euler
import geometry_msgs.msg
from numpy import linspace
import sys

tau = 2 * math.pi 

def create_pose(x, y, z, roll, pitch, yaw):
    pose = geometry_msgs.msg.Pose()
    quat_tf = quaternion_from_euler(roll, pitch, yaw)
    orientation = geometry_msgs.msg.Quaternion(quat_tf[0], quat_tf[1], quat_tf[2], quat_tf[3])
    pose.orientation = orientation
    pose.position.x = x
    pose.position.y = y
    pose.position.z = z
    return pose

def sequence(status, command_name):
    if status == True:
        print(f"Passed {command_name}.")
    else:
        print(f"Failed {command_name}.")
        print("Exiting ...")
        sys.exit()

def demo():
    manipulation = ManipulationPrimitives()

    sequence(manipulation.move_to_target('step6'), "Moving to Home")

    sequence(manipulation.explore_environment(), "Explore Environment")

    sequence(manipulation.move_to_target('step6'), "Moving to Home")
    input("Waiting user confirmation")
    sequence(manipulation.pick("cube", retreat='-x', axis_constraints=['+x', '-x', '+y', '-y', '-z']), "Pick Cube")
    sequence(manipulation.place("cube", create_pose(.3, .1, .1, 0, 0, 0)), "Place Cube")

    sequence(manipulation.move_to_target('step6'), "Moving to Home")
    input("Waiting user confirmation")
    sequence(manipulation.pick("box", retreat='-x', axis_constraints=['+x', '-x', '+y', '-y', '-z']), "Pick Box")
    sequence(manipulation.drop("box", create_pose(.3, -.1, .1, 0, tau/4, 0)), "Drop Box")

    sequence(manipulation.move_to_target('inspect_ground'), "Moving to Home")
    sequence(manipulation.explore_environment(), "Inspecting Environment")

    sequence(manipulation.move_to_target('inspect_ground'), "Moving to Home")
    input("Waiting user confirmation")
    sequence(manipulation.pick("cube", retreat='+z', axis_constraints=['+x', '-x', '+y', '-y', '-z']), "Pick Cube")
    sequence(manipulation.drop("cube", create_pose(.15, -.42, .15, 0, 0, 0)), "Drop Box")

    sequence(manipulation.move_to_target('inspect_ground'), "Moving to Home")
    input("Waiting user confirmation")
    sequence(manipulation.pick("box", retreat='+z', axis_constraints=['+x', '-x', '+y', '-y', '-z']), "Pick Box")
    sequence(manipulation.drop("box", create_pose(.15, -.42, .15, 0, 0, 0)), "Drop Box")

    sequence(manipulation.move_to_target('inspect_ground'), "Moving to Home")

def user_input():
    manipulation = ManipulationPrimitives()

    while True:
        print(
"""
Control Robot
---
0: move_to_pose(x, y, z, roll, pitch, yaw)
1: move_to_target(target)
2: explore_environment()
3: pick(object_id, retreat=None, axis_constraints=[])
4: drop(object_id, pose)
5: drop_at_location(object_id, location)
6: place(self, object_id, pose, delete_object=False)
7: reset()
8: exit
"""
        )

        status = False

        command = input("Enter command: ")

        try:
            command = int(command)
            if command < 0 or command > 8:
                raise ValueError('Invalid Range')
        except:
            print("Invalid Input")
            continue

        if command == 0:
            pose = input("Enter x y z r p y: ")
            try:
                pose = pose.split()
                if len(pose) != 6:
                    raise ValueError('Invalid Number of Arguments')
                pose = [float(x) for x in pose]
            except:
                print("Invalid Input")
                continue
            status = manipulation.move_to_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])

        elif command == 1:
            target = input("Enter target: ")
            try:
                status = manipulation.move_to_target(target)
            except:
                print("Invalid Input")
                continue

        elif command == 2:
            status = manipulation.explore_environment()

        elif command == 3:
            object_id = input("Enter object ID (cube, box, cylinder): ")
            retreat = input("Enter axis to retreat: ")
            axis_constraints = input("Enter axis constraints (+x, -x, +y, -y, +z, -z, s): ").split()
            status = manipulation.pick(object_id, retreat=retreat, axis_constraints=axis_constraints)
            
        elif command == 4:
            input("Enter object ID (cube, box, cylinder): ")
            pose = input("Enter x y z r p y: ")
            try:
                pose = pose.split()
                if len(pose) != 6:
                    raise ValueError('Invalid Number of Arguments')
                pose = [float(x) for x in pose]
                pose = create_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
            except:
                print("Invalid Input")
                continue
            status = manipulation.drop(object_id, pose)
        elif command == 5:
            object_id = input("Enter object ID (cube, box, cylinder): ")
            location = input("Enter Location (A, B, C): ").strip()
            if location == 'C':
                pose = create_pose(.15, -.42, .15, 0, 0, 0)
            elif location == 'B':
                pose = create_pose(.3, 0, .15, 0, tau/4, 0)
            elif location == 'A':
                pose = create_pose(.3, .45, .15, 0, tau/4, tau/8)
            else:
                print("Invalid Input")
                continue
            status = manipulation.drop(object_id, pose)
        elif command == 6:
            object_id = input("Enter object ID (cube, box, cylinder): ")
            pose = input("Enter x y z r p y: ")
            try:
                pose = pose.split()
                if len(pose) != 6:
                    raise ValueError('Invalid Number of Arguments')
                pose = [float(x) for x in pose]
                pose = create_pose(pose[0], pose[1], pose[2], pose[3], pose[4], pose[5])
            except:
                print("Invalid Input")
                continue
            status = manipulation.place(object_id, pose)
        elif command == 7: 
            status = manipulation.reset()
        elif command == 8:
            print("Exiting")
            status = True
            break

        


if __name__ == "__main__":
    rospy.init_node('control')

    user_input()

    moveit_commander.roscpp_shutdown()