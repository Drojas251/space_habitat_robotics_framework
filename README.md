# Space Habitat Robotics Framework

This framework is designed to enable complex manipulation tasks by providing a set of primitive robotic actions that can be sequenced together to execute a task. Such primitive robotic actions are built using Moveit and are accessible either through a Class interface or a ROS action interface. Behavior Trees are leveraged to call and sequence the available ROS Actions in order to execute multi-step tasks. Below is a description of the provided packages:

### shr_manipulation 
Contains three main files:
* __manipulation_primitives.py__: Contains the ManipulationPrimitive Class which uses moveit to build up primitive robot functions. These functions can be used directly by the user to program a robot.  
* __manipulation_action_server.py__: Contains the ManipulationActionServer Class which inherits from the ManipulationPrimitive Class and wrapped the primitive robot functions in a ROS Action Interface. Rather than using the methods in the ManipulationPrimitive Class directly, this Class starts up ROS Action servers for each robot function which can be executed by sending a request to a Action server. 
* __grasp_planner.py__: Contains the GraspPlanner Class which provides the 'grasp_planning_service' ROS2 service that is used to plan grasps for the arms based on prior knowledge and information about the objects of interest. 
* __/launch/bringup_arm.launch__: This is the main launch file to launch moveit, manipulation action servers, grasp planner, and vision nodes. 

### shr_interfaces
Contains action and service message definitions which are used in this repo, along with various .yaml files used for april_tag nodes, and object database.

### shr_vision
Contains a nodes to add objects into the planning scene from database

### shr_behavior_trees
* __/src/nodes__: Contains Behavior Tree nodes for each ROS Action that is made avaible from the ManipulationActionServer Class. Most Behavior Tree nodes contain the ROS Action client required to call a given ROS action server that is spun up by launching the manipulation_action_server.py file. 

* __test_bt.cpp__: Is the main executable which registers each behavior tree node in /src/nodes, loads a .xml file which describes the desired Behavior Tree that is designed by the user, and ticks the tree

* __behavior_tree_server.cpp__: Is a ROS action server which essentially wraps the contents in test_bt.cpp and accepts a string name of a Behavior Tree .xml file inorder to execute different Behavior Trees easily. 

* __/bt_xml__: contains the .xml files that describe a Behavior Tree

* __/launch/my_bt.launch__: Is an example which inputs a .xml file from /bt_xml into test_bt.cpp and executes the Behavior Tree.


