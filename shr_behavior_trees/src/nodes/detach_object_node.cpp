#include "detach_object_node.h"

using namespace BT;

DetachObject::DetachObject(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList DetachObject::providedPorts() {
  return { InputPort<std::string>("object_id") };
}

BT::NodeStatus DetachObject::tick() {
  // Take the goal from the InputPort of the Node
  std::string object_id;

  if (!getInput<std::string>("object_id", object_id)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<shr_interfaces::String>("detach_object");
  shr_interfaces::String srv;
  srv.request.data = object_id;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("DetachObject failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call DetachObject");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("DetachObject succeeded");
  return BT::NodeStatus::SUCCESS;
}