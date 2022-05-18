#include "clear_octomap_node.h"

using namespace BT;

ClearOctomap::ClearOctomap(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList ClearOctomap::providedPorts() {
  return { };
}

BT::NodeStatus ClearOctomap::tick() {
  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<std_srvs::Trigger>("clear_octomap_node");
  std_srvs::Trigger srv;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("ClearOctomap failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call ClearOctomap");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("ClearOctomap succeeded");
  return BT::NodeStatus::SUCCESS;
}