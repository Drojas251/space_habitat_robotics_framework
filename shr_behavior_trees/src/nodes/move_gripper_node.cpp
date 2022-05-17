#include "move_gripper_node.h"

using namespace BT;

MoveGripper::MoveGripper(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config),
  move_gripper_client("move_gripper", true)
{
}

PortsList MoveGripper::providedPorts() {
  return { InputPort<double>("gripper_width") };
}

BT::NodeStatus MoveGripper::tick() {
  // if no server is present, fail after 2 seconds
  if (!move_gripper_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_gripper action server");
    return BT::NodeStatus::FAILURE;
  }

  // Take the goal from the InputPort of the Node
  double gripper_width;

  if (!getInput<double>("gripper_width", gripper_width)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  // Reset this flag
  _aborted = false;

  ROS_INFO("Sending goal");

  shr_interfaces::MoveGripperGoal msg;
  msg.gripper_width = gripper_width;
  move_gripper_client.sendGoal(msg);

  while (!_aborted && !move_gripper_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    //_client.cancelAllGoals();
    ROS_ERROR("MoveGripper aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (move_gripper_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveGripper failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("MoveGripper succeeded");
  return BT::NodeStatus::SUCCESS;
}

void MoveGripper::halt() 
{
  _aborted = true;
}