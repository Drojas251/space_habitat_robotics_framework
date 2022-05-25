#include "nodes/pick_node.h"
#include "nodes/place_node.h"
#include "nodes/move_to_target_node.h"
#include "nodes/move_to_pose_node.h"
#include "nodes/move_gripper_node.h"
#include "nodes/move_gripper_to_target_node.h"
#include "nodes/remove_object_node.h"
#include "nodes/detach_object_node.h"
#include "nodes/clear_octomap_node.h"
#include "nodes/enable_camera_node.h"
#include "nodes/wait_node.h"
#include "nodes/reset_node.h"
#include "nodes/add_object_node.h"
#include <ros/ros.h>

#include <behaviortree_cpp_v3/bt_factory.h>
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include "behaviortree_cpp_v3/behavior_tree.h"

using namespace BT;

int main(int argc, char **argv) {

  ros::init(argc, argv, "test_bt");
  ros::NodeHandle nh("~");

  std::string xml_filename;
  nh.getParam("/bt_xml", xml_filename);
  std::cout << xml_filename << "********" << std::endl;
  ROS_INFO("Loading XML : %s", xml_filename.c_str());

  // We use the BehaviorTreeFactory to register our custom nodes
  BehaviorTreeFactory factory;

  factory.registerNodeType<Pick>("Pick");
  factory.registerNodeType<Place>("Place");
  factory.registerNodeType<MoveToTarget>("MoveToTarget");
  factory.registerNodeType<MoveToPose>("MoveToPose");
  factory.registerNodeType<MoveGripper>("MoveGripper");
  factory.registerNodeType<MoveGripperToTarget>("MoveGripperToTarget");
  factory.registerNodeType<RemoveObject>("RemoveObject");
  factory.registerNodeType<DetachObject>("DetachObject");
  factory.registerNodeType<Reset>("Reset");
  factory.registerNodeType<ClearOctomap>("ClearOctomap");
  factory.registerNodeType<EnableCamera>("EnableCamera");
  factory.registerNodeType<Wait>("Wait");
  factory.registerNodeType<AddObject>("AddObject");


  // Trees are created at deployment-time (i.e. at run-time, but only once at
  // the beginning). The currently supported format is XML. IMPORTANT: when the
  // object "tree" goes out of scope, all the TreeNodes are destroyed
  auto tree = factory.createTreeFromFile(xml_filename);

  // Create a logger
  StdCoutLogger logger_cout(tree);

  NodeStatus status = NodeStatus::RUNNING;
  // Keep on ticking until you get either a SUCCESS or FAILURE state
  while (ros::ok() && status == NodeStatus::RUNNING) {
    status = tree.rootNode()->executeTick();
    // Sleep 100 milliseconds
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }

  return 0;
}