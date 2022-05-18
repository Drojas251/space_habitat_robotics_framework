#include "reset_node.h"

using namespace BT;

Reset::Reset(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList Reset::providedPorts() {
  return { };
}

BT::NodeStatus Reset::tick() {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("reset");
  std_srvs::Trigger srv;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("Reset failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call Reset");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("Reset succeeded");
  return BT::NodeStatus::SUCCESS;
}