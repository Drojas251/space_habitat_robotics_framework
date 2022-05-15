#include <ros/ros.h>

#include <actionlib/client/simple_action_client.h>
#include <actionlib/client/terminal_state.h>

#include <shr_interfaces/PickAction.h>
#include <shr_interfaces/PickGoal.h>
#include <shr_interfaces/PickResult.h>
#include <shr_interfaces/PickFeedback.h>

#include "behaviortree_cpp_v3/behavior_tree.h"
//#include "behaviortree_cpp_v3/bt_factory.h"

#include <std_msgs/Float64.h>

class Pick : public BT::SyncActionNode{
  public:

    Pick(const std::string& name, const BT::NodeConfiguration& config) : 
      BT::SyncActionNode(name, config),
      _client("pick", true)
    { }

    static BT::PortsList providedPorts()
    {
      return{ BT::InputPort<std::string>("object_id") };
    }

    BT::NodeStatus tick() override;

  private:
    typedef actionlib::SimpleActionClient<shr_interfaces::PickAction> PickClient;
    PickClient _client;
};