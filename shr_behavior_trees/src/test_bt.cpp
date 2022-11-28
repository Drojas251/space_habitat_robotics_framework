#include "action_client_nodes.h"
#include "node_input_conversions.h"

#include <ros/ros.h>
#include <iostream>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace BT;

int count = 0;

BT::NodeStatus CheckBattery()
{
  std::cout << count << std::endl;
  if (count < 10000) {
    return BT::NodeStatus::SUCCESS;
  }
  else {
    
    return BT::NodeStatus::FAILURE;
  }
}

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");
  ros::NodeHandle nh("~");

  std::string xml_filename;
  nh.getParam("/bt_xml", xml_filename);
  std::cout << xml_filename << "********" << std::endl;
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerSimpleCondition("CheckBattery", std::bind(CheckBattery));
  factory.registerNodeType<ActionClientNode<MoveToTargetPolicy>>("MoveToTarget");
  factory.registerNodeType<ActionClientNode<MoveToPosePolicy>>("MoveToPose");
  factory.registerNodeType<ActionClientNode<MoveGripperPolicy>>("MoveGripper");
  factory.registerNodeType<ActionClientNode<MoveGripperToTargetPolicy>>("MoveGripperToTarget");
  factory.registerNodeType<ActionClientNode<PreGraspApproachPolicy>>("PreGraspApproach");
  factory.registerNodeType<ActionClientNode<MoveCartesianPathPolicy>>("MoveCartesianPath");

  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    count++;
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}