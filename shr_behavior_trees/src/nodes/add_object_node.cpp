#include "add_object_node.h"

namespace BT
{
  template <> inline geometry_msgs::Pose convertFromString(StringView str)
  {
    // The next line should be removed...
    printf("Converting string: \"%s\"\n", str.data() );

    // We expect real numbers separated by semicolons
    auto parts = splitString(str, ';');
    if (parts.size() == 7) {
      geometry_msgs::Pose pose;
      pose.position.x = convertFromString<double>(parts[0]);
      pose.position.y = convertFromString<double>(parts[1]);
      pose.position.z = convertFromString<double>(parts[2]);
      pose.orientation.x = convertFromString<double>(parts[3]);
      pose.orientation.y = convertFromString<double>(parts[4]);
      pose.orientation.z = convertFromString<double>(parts[5]);
      pose.orientation.w = convertFromString<double>(parts[6]);

      return pose;
    }
    else if (parts.size() == 6) {
      geometry_msgs::Pose pose;
      pose.position.x = convertFromString<double>(parts[0]);
      pose.position.y = convertFromString<double>(parts[1]);
      pose.position.z = convertFromString<double>(parts[2]);

      tf2::Quaternion quat;
      double r = convertFromString<double>(parts[3]);
      double p = convertFromString<double>(parts[4]);
      double y = convertFromString<double>(parts[5]);
      quat.setRPY( r, p, y );
      pose.orientation.x = quat.getX();
      pose.orientation.y = quat.getY();
      pose.orientation.z = quat.getZ();
      pose.orientation.w = quat.getW();

      return pose;
    }
    else {
      throw RuntimeError("invalid input");
    }
  }
}

using namespace BT;

AddObject::AddObject(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config)
{
}

PortsList AddObject::providedPorts() {
  return { 
    InputPort<std::string>("object_id"),
    InputPort<geometry_msgs::Pose>("pose")
  };
}

BT::NodeStatus AddObject::tick() {
  // Take the goal from the InputPort of the Node
  std::string object_id;
  geometry_msgs::Pose pose;
  std::string type;
  std::vector<double> dimensions;

  if (!getInput<std::string>("object_id", object_id)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input object_id");
  }
  if (!getInput<geometry_msgs::Pose>("pose", pose)) {
    throw BT::RuntimeError("missing required input pose");
  }

  ros::NodeHandle n;
  ros::ServiceClient client = n.serviceClient<shr_interfaces::AddObject>("add_object");
  shr_interfaces::AddObject srv;
  srv.request.object_id = object_id;
  srv.request.pose = pose;

  if (client.call(srv))
  {
    if (!srv.response.success) {
      ROS_ERROR("AddObject failed");
      return BT::NodeStatus::FAILURE;
    }
  }
  else
  {
    ROS_ERROR("Failed to call AddObject");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("AddObject succeeded");
  return BT::NodeStatus::SUCCESS;
}