#include "pick_node.h"

using namespace BT;

Pick::Pick(const std::string& name, const BT::NodeConfiguration& config): 
  SyncActionNode(name, config),
  pick_client("pick", true) 
{
}

PortsList Pick::providedPorts() {
  return { InputPort<std::string>("object_id") };
}

BT::NodeStatus Pick::tick() {
  // if no server is present, fail after 2 seconds
  if (!pick_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact pick action server");
    return BT::NodeStatus::FAILURE;
  }

  // Take the goal from the InputPort of the Node
  std::string object_id;

  if (!getInput<std::string>("object_id", object_id)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  ROS_INFO("Sending goal");

  shr_interfaces::PickGoal msg;
  msg.object_id = object_id;
  std::cout << msg.object_id << std::endl;
  pick_client.sendGoal(msg);

  while (!pick_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (pick_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("Pick failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Pick succeeded");
  return BT::NodeStatus::SUCCESS;
}