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
#include <actionlib/server/simple_action_server.h>

#include <shr_interfaces/ExecuteBehaviorTreeAction.h>

using namespace BT;

class BehaviorTreeServer
{
private:

  ros::NodeHandle nh_;
  actionlib::SimpleActionServer<shr_interfaces::ExecuteBehaviorTreeAction> as_; // NodeHandle instance must be created before this line. Otherwise strange error occurs.
  std::string action_name_;
  // create messages that are used to published feedback/result
  shr_interfaces::ExecuteBehaviorTreeFeedback feedback_;
  shr_interfaces::ExecuteBehaviorTreeResult result_;
  BehaviorTreeFactory factory;

public:

  BehaviorTreeServer(std::string name) :
    as_(nh_, name, boost::bind(&BehaviorTreeServer::executeCB, this, _1), false),
    action_name_(name)
  {
    as_.start();

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

  }

  ~BehaviorTreeServer(void)
  {
  }

  void executeCB(const shr_interfaces::ExecuteBehaviorTreeGoalConstPtr &goal)
  {
    // helper variables
    ros::Rate r(1);
    bool success = true;

    std::string xml = goal->xml;

    // Trees are created at deployment-time (i.e. at run-time, but only once at
    // the beginning). The currently supported format is XML. IMPORTANT: when the
    // object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml);

    // Create a logger
    StdCoutLogger logger_cout(tree);

    NodeStatus status = NodeStatus::RUNNING;
    // Keep on ticking until you get either a SUCCESS or FAILURE state
    while (ros::ok() && status == NodeStatus::RUNNING) {

        if (as_.isPreemptRequested() || !ros::ok())
        {
            ROS_INFO("%s: Preempted", action_name_.c_str());
            // set the action state to preempted
            as_.setPreempted();
            success = false;
            break;
        }

        status = tree.rootNode()->executeTick();
        // Sleep 100 milliseconds
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }

    if(success)
    {
      as_.setSucceeded();
    }
  }


};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "behavior_tree_server");

  BehaviorTreeServer server("execute_behavior_tree");
  ros::spin();

  return 0;
}