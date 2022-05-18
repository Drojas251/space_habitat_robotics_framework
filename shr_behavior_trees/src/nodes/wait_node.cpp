#include "wait_node.h"

using namespace BT;

Wait::Wait(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList Wait::providedPorts() {
  return { InputPort<double>("sec") };
}

BT::NodeStatus Wait::tick() {
  // Take the goal from the InputPort of the Node
  double sec;

  if (!getInput<double>("sec", sec)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<shr_interfaces::Float>("wait");
  shr_interfaces::Float srv;
  srv.request.data = sec;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("Wait failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call Wait");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Wait succeeded");
  return BT::NodeStatus::SUCCESS;
}