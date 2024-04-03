#include <cstdio>
#include <iostream>
#include <sstream>

#include "dynrclcpp/node_base.hpp"

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


    auto callback_ = std::bind(&ExampleNode::callback, this, std::placeholders::_1);
    
    // TEST SUBSCRIPTIONS
    auto sub1 = node->create_subscription("/joy", "sensor_msgs/msg/Joy", rmw_qos_profile_default, callback_);
    auto sub2 = node->create_subscription("/chatter", "std_msgs/msg/String", rmw_qos_profile_default, callback_);
    auto sub3 = node->create_subscription("/chatter", rmw_qos_profile_default, callback_);
   
    if(sub1 != nullptr) sub1->subscribe();
    if(sub2 != nullptr) sub2->subscribe();
    if(sub3 != nullptr) sub3->subscribe();

    // TEST PUBLISHER
    auto pub1 = node->create_publisher("test_pub1", "std_msgs/msg/Int32", rmw_qos_profile_sensor_data);
    std::string msg1 = "data: 10";
    
    // TEST TIMER
    auto tim1_callback = std::bind(&ExampleNode::timer_callback, this);
    auto tim1 = node->create_timer("pub1_tim", tim1_callback, std::chrono::seconds(1));

    std::this_thread::sleep_for(std::chrono::seconds(5));
    node->destroy_subscription("/chatter", "std_msgs/msg/String");

 
    node->spin();
    
  }

  void timer_callback(){
      std::cout << "hello "<< std::endl;
  }

  void callback(RosMessage msg){
    std::cout << dynmsg::c::message_to_yaml(msg) << "\n";
    
  }


private:


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
