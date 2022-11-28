#ifndef ACTION_CLIENT_NODE_TEMPLATE_H
#define ACTION_CLIENT_NODE_TEMPLATE_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace BT;

template <class ActionClientPolicy>
class ActionClientNode : public ActionClientPolicy
{
  public:
    // If your Node has ports, you must use this constructor signature 
    ActionClientNode(const std::string& name, const BT::NodeConfiguration& config);

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;

    virtual void halt() override;

  private:
    bool _aborted;
};

template <class ActionClientPolicy>
ActionClientNode<ActionClientPolicy>::ActionClientNode(const std::string& name, const BT::NodeConfiguration& config) :
ActionClientPolicy(name, config)
{
}

template <class ActionClientPolicy>
BT::NodeStatus ActionClientNode<ActionClientPolicy>::tick() {
  // if no server is present, fail after 2 seconds
  if (!ActionClientPolicy::action_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact action server");
    return BT::NodeStatus::FAILURE;
  }

  // Reset this flag
  _aborted = false;

  ActionClientPolicy::sendGoal();

  while (!_aborted && !ActionClientPolicy::action_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    ActionClientPolicy::action_client.cancelAllGoals();
    ROS_ERROR("aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (ActionClientPolicy::action_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("succeeded");
  return BT::NodeStatus::SUCCESS;
}

template <class ActionClientPolicy>
void ActionClientNode<ActionClientPolicy>::halt() 
{
  _aborted = true;
}

#endif