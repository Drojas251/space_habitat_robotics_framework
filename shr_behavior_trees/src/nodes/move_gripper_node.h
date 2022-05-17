#ifndef MOVE_GRIPPER_NODE_H
#define MOVE_GRIPPER_NODE_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <shr_interfaces/MoveGripperAction.h>
#include <shr_interfaces/MoveGripperGoal.h>
#include <shr_interfaces/MoveGripperResult.h>
#include <shr_interfaces/MoveGripperFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveGripper : public BT::AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    MoveGripper(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    actionlib::SimpleActionClient<shr_interfaces::MoveGripperAction> move_gripper_client;
    bool _aborted;
};

#endif