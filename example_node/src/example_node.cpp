#include <cstdio>
#include <iostream>
#include <sstream>

#include "dynrclcpp/node_base.hpp"
#include "dynrclcpp/timer_base.hpp"

#include <thread>
#include <chrono>
#include <mutex>

#include "rcutils/logging_macros.h"
#include "rcutils/logging.h"

#include "dynmsg/message_reading.hpp"




std::mutex mtx_;

class ExampleNode{
public:
  ExampleNode(const std::string& name, int argc, char** argv){

    std::shared_ptr<dynrclcpp::NODE> node = std::make_shared<dynrclcpp::NODE>(name);
    node->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
    node->init(argc, argv);
    
    /// TEST SUBSCRIPTIONS
    auto callback_ = std::bind(&ExampleNode::sub_callback, this, std::placeholders::_1);
    // auto sub1 = node->create_subscription("/joy", "sensor_msgs/msg/Joy", rmw_qos_profile_default, callback_);
    auto sub2 = node->create_subscription("/chatter", "std_msgs/msg/String", rmw_qos_profile_default, callback_);
    // auto sub3 = node->create_subscription("/chatter", rmw_qos_profile_default, callback_);
   
    // if(sub1 != nullptr) sub1->subscribe();
    if(sub2 != nullptr) sub2->subscribe();
    // if(sub3 != nullptr) sub3->subscribe();
   
    /// TEST TIMER WITH A PUBLISHER
    pub1 = node->create_publisher("test_pub1", "std_msgs/msg/Int32", rmw_qos_profile_sensor_data);
    auto tim1_callback = std::bind(&ExampleNode::timer_callback, this);
    if(pub1 != nullptr) pub1->timer = node->create_timer("pub1_tim", tim1_callback, std::chrono::seconds(1));


    /// TEST DESTROY
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // node->destroy_subscription("/chatter", "std_msgs/msg/String");
    // node->destroy_publisher("test_pub1", "std_msgs/msg/Int32"); // Timer will be automatically destroyed

    /// TEST ROS GRAPH
    nlohmann::json nodeGraph = node->get_nodes_info();
    std::string nodeGrpah_str = nodeGraph.dump();
    RCUTILS_LOG_INFO_NAMED(node->node_name.c_str(), nodeGrpah_str.c_str() );
    
    


    // use spin only if you dont use any other process except the node
    node->spin();
    
  }

  void timer_callback(){
    std::string msg1 = "data: 10";
    pub1->publish(msg1);
  }

  void sub_callback(RosMessage msg){
    std::cout << dynmsg::c::message_to_yaml(msg) << "\n";
    
  }


private:
  std::shared_ptr<DynPublisher> pub1;
  

};




int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  ExampleNode node("example_node", argc, argv);

  while(true){
    // RCUTILS_LOG_INFO_NAMED("example node", "main thread");
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }

 
  printf("hello world example_node package\n");
  return 0;
}
