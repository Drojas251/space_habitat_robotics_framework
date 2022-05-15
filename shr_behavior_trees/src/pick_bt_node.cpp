#include <ros/ros.h>


#include "behaviortree_cpp_v3/bt_factory.h"
#include "pick_bt_node.h"


BT::NodeStatus Pick::tick() {
  // if no server is present, fail after 2 seconds
  if (!_client.waitForServer(ros::Duration(2.0))) {
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

  ROS_INFO("Sending goal ");

  shr_interfaces::PickGoal msg;

  msg.object_id = object_id;

  _client.sendGoal(msg);

  while (!_client.waitForResult(ros::Duration(0.02))) {
  // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
  ROS_ERROR("Pick failed");
  return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Pick succeeded");
  return BT::NodeStatus::SUCCESS;
}