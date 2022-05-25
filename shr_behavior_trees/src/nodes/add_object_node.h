#ifndef ADD_OBJECT_NODE_H
#define ADD_OBJECT_NODE_H

#include <ros/ros.h>

#include <shr_interfaces/AddObject.h>
#include <geometry_msgs/Pose.h>
#include <tf2/LinearMath/Quaternion.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class AddObject : public BT::AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    AddObject(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;
};

#endif