#include "enable_camera_node.h"

using namespace BT;

EnableCamera::EnableCamera(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList EnableCamera::providedPorts() {
  return { InputPort<bool>("enable") };
}

BT::NodeStatus EnableCamera::tick() {
  // Take the goal from the InputPort of the Node
  bool enable;

  if (!getInput<bool>("enable", enable)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::SetBool>("enable_camera");
  std_srvs::SetBool srv;
  srv.request.data = enable;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("EnableCamera failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call EnableCamera");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("EnableCamera succeeded");
  return BT::NodeStatus::SUCCESS;
}