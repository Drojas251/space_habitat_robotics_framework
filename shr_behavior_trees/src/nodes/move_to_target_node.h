#ifndef MOVE_TO_TARGET_NODE_H
#define MOVE_TO_TARGET_NODE_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <shr_interfaces/StringAction.h>
#include <shr_interfaces/StringGoal.h>
#include <shr_interfaces/StringResult.h>
#include <shr_interfaces/StringFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class MoveToTarget : public BT::AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    MoveToTarget(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    actionlib::SimpleActionClient<shr_interfaces::StringAction> move_to_target_client;
    bool _aborted;
};

#endif