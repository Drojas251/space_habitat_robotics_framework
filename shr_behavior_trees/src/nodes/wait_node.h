#ifndef WAIT_NODE_H
#define WAIT_NODE_H

#include <ros/ros.h>

#include <shr_interfaces/Float.h>

#include "behaviortree_cpp_v3/behavior_tree.h"

class Wait : public BT::AsyncActionNode
{
  public:
    // If your Node has ports, you must use this constructor signature 
    Wait(const std::string& name, const BT::NodeConfiguration& config);

    // It is mandatory to define this static method.
    static BT::PortsList providedPorts();

    // As usual, you must override the virtual function tick()
    BT::NodeStatus tick() override;
};

#endif