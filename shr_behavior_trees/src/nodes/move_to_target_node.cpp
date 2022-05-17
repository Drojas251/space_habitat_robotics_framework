#include "move_to_target_node.h"

using namespace BT;

MoveToTarget::MoveToTarget(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config),
  move_to_target_client("move_to_target", true)
{
}

PortsList MoveToTarget::providedPorts() {
  return { InputPort<std::string>("target") };
}

BT::NodeStatus MoveToTarget::tick() {
  // if no server is present, fail after 2 seconds
  if (!move_to_target_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_to_target action server");
    return BT::NodeStatus::FAILURE;
  }

  // Take the goal from the InputPort of the Node
  std::string target;

  if (!getInput<std::string>("target", target)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  // Reset this flag
  _aborted = false;

  ROS_INFO("Sending goal");

  shr_interfaces::MoveToTargetGoal msg;
  msg.target = target;
  move_to_target_client.sendGoal(msg);

  while (!_aborted && !move_to_target_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MoveToTarget aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (move_to_target_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveToTarget failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("MoveToTarget succeeded");
  return BT::NodeStatus::SUCCESS;
}

void MoveToTarget::halt() 
{
  _aborted = true;
}