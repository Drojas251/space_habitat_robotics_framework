#ifndef PICK_NODE_H
#define PICK_NODE_H

#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <shr_interfaces/PickAction.h>
#include <shr_interfaces/PickGoal.h>
#include <shr_interfaces/PickResult.h>
#include <shr_interfaces/PickFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class Pick : public BT::SyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    Pick(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;

  private:
    actionlib::SimpleActionClient<shr_interfaces::PickAction> pick_client;
};

#endif