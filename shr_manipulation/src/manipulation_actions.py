#! /usr/bin/env python3

import rospy
import moveit_commander
import math
import moveit_msgs.msg
import geometry_msgs.msg
import actionlib_msgs.msg
import tf.transformations
import actionlib
from shr_interfaces.msg import \
    PlanAction, PlanGoal

tau = 2 * math.pi 

def qv_mult(q1, v1):
    q2 = list(v1)
    q2.append(0.0)
    return tf.transformations.quaternion_multiply(
        tf.transformations.quaternion_multiply(q1, q2), 
        tf.transformations.quaternion_conjugate(q1)
    )[:3]    

class ManipulationActions:
    def __init__(self, move_groups):
        self.move_group = {}
        for group in move_groups:
            self.move_group[group] = moveit_commander.MoveGroupCommander(group)
            self.move_group[group].set_max_velocity_scaling_factor(1.0)
            self.move_group[group].set_max_acceleration_scaling_factor(1.0)

        self.scene = moveit_commander.PlanningSceneInterface()
        self.robot = moveit_commander.RobotCommander()

        # action server or asynchronous plan and execute
        self.execute_plan_as = actionlib.SimpleActionServer(
            'execute_plan', 
            PlanAction, 
            execute_cb=self.execute_plan_cb, 
            auto_start=False
        )
        self.execute_plan_as.start()

        # action client to send trajectories to plan and execute server
        self.execute_plan_client = actionlib.SimpleActionClient('execute_plan', PlanAction)
        self.execute_plan_client.wait_for_server()

    # Moves to a Pose
    def move_to_pose(self, group_name, pose):
        group = self.move_group[group_name]
        group.set_pose_target(pose)

        error_code_val, plan, planning_time, error_code = group.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        self.async_execute_plan(group_name, plan)
        return True

    # Moves to a target joint position
    def move_to_target(self, group_name, target):
        group = self.move_group[group_name]
        group.set_named_target(target)

        error_code_val, plan, planning_time, error_code = group.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        self.async_execute_plan(group_name, plan)
        return True

    # Moves gripper to joint position
    def move_gripper(self, group_name, pos): 
        group = self.move_group[group_name]
        joint_goal = group.get_current_joint_values()
        joint_goal[0] = pos

        group.set_joint_value_target(joint_goal)

        error_code_val, plan, planning_time, error_code = group.plan()
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            return False

        self.async_execute_plan(group_name, plan)
        return True

    def pregrasp_approach(self, group_name, distance):
        group = self.move_group[group_name]

        pose = group.get_current_pose().pose

        q = pose.orientation
        offset = qv_mult(
            [q.x, q.y, q.z, q.w], 
            [distance, 0, 0]
        )

        pose.position.x += offset[0]
        pose.position.y += offset[1]
        pose.position.z += offset[2]

        success = self.move_cartesian_path(group_name, pose)
        return success

    def move_cartesian_path(self, group_name, pose):
        group = self.move_group[group_name]

        waypoints = []
        waypoints.append(pose)

        (plan, fraction) = group.compute_cartesian_path(
            waypoints,
            eef_step=0.01,
            jump_threshold=0, # 0?
            avoid_collisions=True
        )

        if fraction < 1:
            return False
        
        self.async_execute_plan(group_name, plan)
        return True

    def stop(self):
        for group in self.move_group.values():
            group.stop()

    def execute_plan_cb(self, msg):
        group = self.move_group[msg.group]
        plan = msg.plan

        error_code_val = group.execute(plan, wait=True)
        success = (error_code_val == moveit_msgs.msg.MoveItErrorCodes.SUCCESS)
        if not success:
            self.execute_plan_as.set_aborted()
            return

        self.execute_plan_as.set_succeeded() 
    
    def async_execute_plan(self, group_name, plan):
        goal = PlanGoal()
        goal.group = group_name
        goal.plan = plan
        self.execute_plan_client.send_goal(goal)

    def wait_for_result(self, actserv):
        while not self.execute_plan_client.wait_for_result(rospy.Duration(0.02)):
            if actserv.is_preempt_requested():
                self.stop()
                actserv.set_preempted()
                return

        if self.execute_plan_client.get_state() != actionlib_msgs.msg.GoalStatus.SUCCEEDED:
            actserv.set_aborted()
            return

        actserv.set_succeeded()

    

