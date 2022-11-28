#include "move_to_pose_node.h"

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

MoveToPose::MoveToPose(const std::string& name, const BT::NodeConfiguration& config): 
  AsyncActionNode(name, config),
  move_to_pose_client("move_to_pose", true)
{
}

PortsList MoveToPose::providedPorts() {
  return { InputPort<geometry_msgs::Pose>("pose") };
}

BT::NodeStatus MoveToPose::tick() {
  // if no server is present, fail after 2 seconds
  if (!move_to_pose_client.waitForServer(ros::Duration(2.0))) {
    ROS_ERROR("Can't contact move_to_pose action server");
    return BT::NodeStatus::FAILURE;
  }

  // Take the goal from the InputPort of the Node
  geometry_msgs::Pose pose;

  if (!getInput<geometry_msgs::Pose>("pose", pose)) {
    // if I can't get this, there is something wrong with your BT.
    // For this reason throw an exception instead of returning FAILURE
    throw BT::RuntimeError("missing required input [goal]");
  }

  // Reset this flag
  _aborted = false;

  ROS_INFO("Sending goal");

  shr_interfaces::MoveToPoseGoal msg;
  msg.pose = pose;
  move_to_pose_client.sendGoal(msg);

  while (!_aborted && !move_to_pose_client.waitForResult(ros::Duration(0.02))) {
    // polling at 50 Hz. No big deal in terms of CPU
  }

  if (_aborted) {
    // this happens only if method halt() was invoked
    move_to_pose_client.cancelAllGoals();
    ROS_ERROR("MoveToPose aborted");
    return BT::NodeStatus::FAILURE;
  }

  if (move_to_pose_client.getState() != actionlib::SimpleClientGoalState::SUCCEEDED) {
    ROS_ERROR("MoveToPose failed");
    return BT::NodeStatus::FAILURE;
  }

  ROS_INFO("MoveToPose succeeded");
  return BT::NodeStatus::SUCCESS;
}

void MoveToPose::halt() 
{
  _aborted = true;
}