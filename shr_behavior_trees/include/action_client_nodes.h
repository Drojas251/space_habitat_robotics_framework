#ifndef ACTION_CLIENT_NODES_H
#define ACTION_CLIENT_NODES_H

#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

#include "action_client_node_template.h"

#include <geometry_msgs/Pose.h>

#include <shr_interfaces/PoseAction.h>
#include <shr_interfaces/PoseGoal.h>
#include <shr_interfaces/StringAction.h>
#include <shr_interfaces/StringGoal.h>
#include <shr_interfaces/FloatAction.h>
#include <shr_interfaces/FloatGoal.h>

class MoveToPosePolicy : public BT::AsyncActionNode
{
public:
  MoveToPosePolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("move_to_pose"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::Pose>("pose") };
  }

  void sendGoal() {
    geometry_msgs::Pose pose;
    if (!getInput<geometry_msgs::Pose>("pose", pose)) {
      throw BT::RuntimeError("missing required input [pose]");
    }
    shr_interfaces::PoseGoal msg;
    msg.pose = pose;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::PoseAction> action_client;
};

class MoveToTargetPolicy : public BT::AsyncActionNode
{
public:
  MoveToTargetPolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("move_to_target"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target") };
  }

  void sendGoal() {
    std::string target;
    if (!getInput<std::string>("target", target)) {
      throw BT::RuntimeError("missing required input [target]");
    }
    shr_interfaces::StringGoal msg;
    msg.string = target;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::StringAction> action_client;
};

class MoveGripperPolicy : public BT::AsyncActionNode
{
public:
  MoveGripperPolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("move_gripper"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<float>("value") };
  }

  void sendGoal() {
    float value;
    if (!getInput<float>("value", value)) {
      throw BT::RuntimeError("missing required input [float]");
    }
    shr_interfaces::FloatGoal msg;
    msg.value = value;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::FloatAction> action_client;
};

class MoveGripperToTargetPolicy : public BT::AsyncActionNode
{
public:
  MoveGripperToTargetPolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("move_gripper_to_target"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<std::string>("target") };
  }

  void sendGoal() {
    std::string target;
    if (!getInput<std::string>("target", target)) {
      throw BT::RuntimeError("missing required input [target]");
    }
    shr_interfaces::StringGoal msg;
    msg.string = target;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::StringAction> action_client;
};

class PreGraspApproachPolicy : public BT::AsyncActionNode
{
public:
  PreGraspApproachPolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("pregrasp_approach"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<float>("value") };
  }

  void sendGoal() {
    float value;
    if (!getInput<float>("value", value)) {
      throw BT::RuntimeError("missing required input [float]");
    }
    shr_interfaces::FloatGoal msg;
    msg.value = value;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::FloatAction> action_client;
};

class MoveCartesianPathPolicy : public BT::AsyncActionNode
{
public:
  MoveCartesianPathPolicy(const std::string& name, const BT::NodeConfiguration& config): 
  server_name("move_cartesian_path"),
  action_client(server_name, true),
  AsyncActionNode(name, config)
  {}

  static BT::PortsList providedPorts() {
    return { BT::InputPort<geometry_msgs::Pose>("pose") };
  }

  void sendGoal() {
    geometry_msgs::Pose pose;
    if (!getInput<geometry_msgs::Pose>("pose", pose)) {
      throw BT::RuntimeError("missing required input [pose]");
    }
    shr_interfaces::PoseGoal msg;
    msg.pose = pose;
    action_client.sendGoal(msg);
  }

  std::string server_name;
  BT::PortsList ports;
  actionlib::SimpleActionClient<shr_interfaces::PoseAction> action_client;
};

#endif