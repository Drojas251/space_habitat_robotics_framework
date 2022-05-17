#ifndef MOVE_TO_POSE_NODE_H
#define MOVE_TO_POSE_NODE_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <shr_interfaces/MoveToPoseAction.h>
#include <shr_interfaces/MoveToPoseGoal.h>
#include <shr_interfaces/MoveToPoseResult.h>
#include <shr_interfaces/MoveToPoseFeedback.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveToPose : public BT::AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    MoveToPose(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    actionlib::SimpleActionClient<shr_interfaces::MoveToPoseAction> move_to_pose_client;
    bool _aborted;
};

#endif