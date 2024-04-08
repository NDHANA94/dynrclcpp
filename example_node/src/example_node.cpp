/* -----------------------------------------------------------------------------
  Copyright 2024 Nipun Dhananjaya Weerakkodi

  Licensed under the Apache License, Version 2.0 (the "License");
  you may not use this file except in compliance with the License.
  You may obtain a copy of the License at

      http://www.apache.org/licenses/LICENSE-2.0

  Unless required by applicable law or agreed to in writing, software
  distributed under the License is distributed on an "AS IS" BASIS,
  WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
  See the License for the specific language governing permissions and
  limitations under the License.
-----------------------------------------------------------------------------*/ 


#include <iostream>
#include <memory>
#include "dynrclcpp/node_base.hpp"




class ExampleNode{
public:
  ExampleNode(const std::string& name, int argc, char** argv){

    node = std::make_shared<dynrclcpp::NODE>(name);
    node->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
    node->init(argc, argv);


    
    /// TEST SUBSCRIPTIONS -----------------------------------------------------------------------------------------
    // auto callback_ = std::bind(&ExampleNode::sub_callback, this, std::placeholders::_1);
    // auto sub1 = node->create_subscription("/joy", "sensor_msgs/msg/Joy", rmw_qos_profile_default, callback_);
    // auto sub2 = node->create_subscription("/chatter", "std_msgs/msg/String", rmw_qos_profile_default, callback_);
    // auto sub3 = node->create_subscription("/chatter", rmw_qos_profile_default, callback_);
    
    // if(sub1 != nullptr) sub1->subscribe();
    // if(sub2 != nullptr) sub2->subscribe();
    // if(sub3 != nullptr) sub3->subscribe();


   
    /// TEST TIMER WITH A PUBLISHER ---------------------------------------------------------------------------------
    pub1 = node->create_publisher("test_pub1", "std_msgs/msg/Int32MultiArray", QOS_DEFAULT);

    if(pub1 != nullptr) pub1->timer = node->create_timer("pub1_tim", std::bind(&ExampleNode::timer_callback, this), std::chrono::seconds(1));

    // TEST CLIENT -------------------------------------------------------------------------------------------------
    client = node->create_client("add_two_ints", "example_interfaces/srv/AddTwoInts", rmw_qos_profile_services_default, 
                  std::bind(&ExampleNode::client_callback, this, std::placeholders::_1));

    YAML::Node req;
    req["a"] = int64_t(10);
    req["b"] = int64_t(10);
    client->response_timeout = std::chrono::milliseconds(10000); // try 10 sec to get response for the request
    client->ignore_timeout_ = true;
    client->send_request(req);

    /// TEST DESTROY -----------------------------------------------------------------------------------------------
    std::this_thread::sleep_for(std::chrono::seconds(3));
    // node->destroy_subscription("/chatter", "std_msgs/msg/String");
    node->destroy_publisher("test_pub1", "std_msgs/msg/Int32"); // Timer will be automatically destroyed
    node->destroy_client("add_two_ints");
    /// TEST NODES INFO
    nlohmann::json nodesInfo = node->get_nodes_info();
    std::string nodesInfo_str = nodesInfo.dump();
    RCUTILS_LOG_INFO_NAMED(node->node_name.c_str(), nodesInfo_str.c_str() );
    
    

    
    
    
  }

  void timer_callback(){
    YAML::Node msg;
    std::vector<int> dataArray = {1,2,3,4,5};
    msg["data"] = dataArray; 
    pub1->publish(msg);
  }

  void sub_callback(RosMessage msg){
    std::cout << dynmsg::c::message_to_yaml(msg) << "\n";
    
  }

  void client_callback(RosSrvResponse msg){
    std::cout << "client recieved response" << "\n";
    
  }

  std::shared_ptr<dynrclcpp::NODE> node;
  std::shared_ptr<dynrclcpp::Publisher> pub1;
  std::shared_ptr<dynrclcpp::Client> client;



};




int main(int argc, char ** argv)
{
  (void) argc;
  (void) argv;


  ExampleNode node("example_node", argc, argv);

  dynrclcpp::spin(); // use spin only if you dont use any other loop process to keep node alive


 
  printf("hello world example_node package\n");
  return 0;
}
