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


#include <stdio.h>

#include "dynrclcpp/node_base.hpp"

#include "rcl/graph.h"

// #include "rcl/rcl.h"


namespace dynrclcpp{

void NODE::init(int argc, char** argv)
{
  // Initialize ROS options
  init_option = rcl_get_zero_initialized_init_options();
  allocator = rcl_get_default_allocator();
  if(rcl_init_options_init(&init_option, allocator) != RCL_RET_OK){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "options init failed: %s", rcl_get_error_string().str);
    // clear up
    if(rcl_init_options_fini(&init_option) != RCL_RET_OK){
      RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "options fini failed: %s", rcl_get_error_string().str);
    }
    return;
  }

  // Initialize ROS
  context = rcl_get_zero_initialized_context();
  if(rcl_init(argc, argv, &init_option, &context) != RCL_RET_OK){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "rcl init failed: %s", rcl_get_error_string().str);
    // clear up
    if( rcl_context_fini(&context) != RCL_RET_OK){
      RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "context fini failed: %s", rcl_get_error_string().str);
    }
    if(rcl_init_options_fini(&init_option) != RCL_RET_OK){
      RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "options fini failed: %s", rcl_get_error_string().str);
    }
    return;
  }

  // create a node
  node_options = rcl_node_get_default_options();
  node = rcl_get_zero_initialized_node();

  if(rcl_node_init(&node, node_name.c_str(), "", &context, &node_options) != RCL_RET_OK){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "node init failed: %s", rcl_get_error_string().str);

    // clear up
    destroy_node();
    return;
  }

  RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "node is successfully created");
}



// destroy the node and its entities
void NODE::destroy_node()
{
  // clearup all the subscriptions
  for (auto& sub : sub_registry){
    sub.second->destroy();
  }
  // clear up all the publishers
  for(auto& pub : pub_registry){
    pub.second->destroy();
  }
  // stop all the timers
  for(auto& tim : timer_registry){
    if (tim.second != nullptr)
      tim.second->stop();
  }

  // clear up node
  if(rcl_node_options_fini(&node_options) != RCL_RET_OK)
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "node options fini failed: %s", rcl_get_error_string().str);
  
  if( rcl_node_fini(&node) != RCL_RET_OK)
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "node fini failed: %s", rcl_get_error_string().str);
 
  if(rcl_shutdown(&context) != RCL_RET_OK)
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "rcl shutdown failed: %s", rcl_get_error_string().str);

  if( rcl_context_fini(&context) != RCL_RET_OK)
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "context fini failed: %s", rcl_get_error_string().str);
  
  if(rcl_init_options_fini(&init_option) != RCL_RET_OK)
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "options fini failed: %s", rcl_get_error_string().str);

  shutdown = true;
  
}


void NODE::change_domain_id(int id)
{
  if (domain_id != id){
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "changing DOMAIN_ID as %d", id);
    destroy_node();
    domain_id = domain_id;
    setenv("ROS_DOMAIN_ID", std::to_string(domain_id).c_str(), 1);
    init(0, NULL);
  }
}


void NODE::set_debug_severity(RCUTILS_LOG_SEVERITY log_type){
  RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "set log level");
  rcutils_ret_t rcu_ret_ = rcutils_logging_set_logger_level("", log_type);
  if(rcu_ret_ != RCUTILS_RET_OK){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "failed to set log level");
  }
}




std::shared_ptr<Subscription> NODE::create_subscription(
const std::string& topic_,
const std::string& type_,
rmw_qos_profile_t qos_,
std::function<void(const YAML::Node msg)> callback_)
{
  
  // need to sleep for a bit for discovery to populate the ROS graph information 
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try{

    // check if the subscription is already available
    auto it = sub_registry.find(topic_);
    if(it != sub_registry.end()){
      throw std::runtime_error("subscription already exists.");
    }

    // create new subscription
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "creating subscription: '%s' ", topic_.c_str());
    std::shared_ptr<Subscription> sub 
      = std::make_shared<Subscription>(topic_, type_, &node, qos_, callback_);
    
    // insert created subscription into subs_registry
    sub_registry.insert({topic_, sub});
    RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "sub is added to subs_registry as key {%s, %s}", topic_.c_str(), type_.c_str());
    return sub;
  } 
  
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Failed to create subscription '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  
}



std::shared_ptr<Subscription> NODE::create_subscription(
const std::string& topic_,
rmw_qos_profile_t qos_,
std::function<void(const YAML::Node msg)> callback_)
{
  
  // need to sleep for a bit for discovery to populate the ROS graph information 
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try{
    // check if the subscription is already available
    auto it = sub_registry.find(topic_);
    if(it != sub_registry.end()){
      throw std::runtime_error("subscription already exists.");
    }

    auto type = get_topic_type_string(&node, topic_);

    // create new subscription
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "creating subscription: '%s' ", topic_.c_str());
    std::shared_ptr<Subscription> sub 
      = std::make_shared<Subscription>(topic_, type, &node, qos_, callback_);
    
    // insert created subscription into subs_registry
    sub_registry.insert({topic_, sub});
    RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "sub is added to subs_registry as key {%s, %s}", topic_.c_str(), type.c_str());
    return sub;
  } 
  
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Falied to create subscription '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  
}



void NODE::destroy_subscription(const std::string& topic_, const std::string& type_)
{
  try{
    // destroy the subscriber
    auto it = sub_registry.find(topic_);
    if(it != sub_registry.end()){
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "destroying subscription...");
      auto sub_to_fini = it->second;
      sub_registry.erase(topic_);
      sub_to_fini->destroy();
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "['%s' | %s ] subscription is finilized", topic_.c_str(), type_.c_str());
      
    }
    else{
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "subscription is not found to destroy");
    }
      
  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error: %s: %s", topic_.c_str(), e.what());
  }
  
}



std::shared_ptr<Publisher> NODE::create_publisher(
const std::string& topic_,
const std::string& type_,
rmw_qos_profile_t qos_)
{
  try{
    std::shared_ptr<Publisher> pub = std::make_shared<Publisher>(topic_, type_, &node, qos_);
    pub_registry.insert({topic_, pub});
    return pub;
  }
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Failed to create publisher '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  

}

void NODE::destroy_publisher(const std::string& topic_, const std::string& type_){
  try{
    // destroy the subscriber
    auto it = pub_registry.find(topic_);
    if(it != pub_registry.end()){
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "destroying subscription...");
      auto sub_to_fini = it->second;
      pub_registry.erase(topic_);
      sub_to_fini->destroy();
      RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "[ '%s' | %s ] publisher is finilized", topic_.c_str(), type_.c_str());
    }
    else{
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "publisher is not found to destroy");
    }
      
  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error: %s: %s", topic_.c_str(), e.what());
  }
}



void NODE::destroy_timer(const std::string& key){
  try{
    if(timer_registry.count(key) == 0){
      std::string err = "'" + key + "' not found";
      throw std::runtime_error(err);
    }
    timer_registry[key]->stop();
    timer_registry.erase(key);
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "timer '%s' is finilized", key.c_str());
    
  } 
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Failed to destroy timer '%s':",   e.what());
  }
}


std::shared_ptr<Client>  NODE::create_client(
const std::string& name, 
const std::string& type,
rmw_qos_profile_t qos,
std::function<void(const YAML::Node res)> callback)
{
  try{
    std::shared_ptr<Client> client = std::make_shared<Client>(&node, name, type, qos, callback);
    client_registry.insert({name, client});
    return client;
  }
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Failed to create client  '%s': %s", name.c_str(), e.what());
    return nullptr;
  }

}


void NODE::destroy_client(const std::string& name){
  auto it = client_registry.find(name);
  if(it != client_registry.end()){
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "Destroying client '%s'", name.c_str());
    it->second->destroy();
    client_registry.erase(name);
  }
  else{
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "Failed to find '%s' client to destroy", name.c_str());
  }
}

std::shared_ptr<Service> NODE::create_service(
const std::string& name, 
const std::string& type,
rmw_qos_profile_t qos,
std::function<void(const YAML::Node req, YAML::Node& res)> callback)
{
  try{
    std::shared_ptr<Service> service = std::make_shared<Service>(&node, name, type, qos, callback);
    service_registry.insert({name, service});
    return service;
  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Failed to create service  '%s': %s", name.c_str(), e.what());
    return nullptr;
  } 
}


void NODE::destroy_service(const std::string& name){
  auto it = service_registry.find(name);
  if(it != service_registry.end()){
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "Destroying client '%s'", name.c_str());
    it->second->destroy();
    service_registry.erase(name);
  }
  else{
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "Failed to find '%s' client to destroy", name.c_str());
  }
}





// ==============================================

nlohmann::json NODE::get_nodes_info(){
  try{
    std::this_thread::sleep_for(std::chrono::seconds(2));
    
    auto node_names = rcutils_get_zero_initialized_string_array();
    auto node_namespaces = rcutils_get_zero_initialized_string_array();
    auto ret = rcl_get_node_names(&node, rcl_get_default_allocator(), &node_names, &node_namespaces);
    if(ret != RCL_RET_OK){
      std::string err = "Failed to get node names: " + std::string(rcl_get_error_string().str);
      throw std::runtime_error(err.c_str());
    }

    // get entities info of each node
    nlohmann::json json_nodeGraph;

    for(size_t i = 0; i < node_names.size; i++){
      if(std::string(node_names.data[i]) == "_ros2cli_daemon_0") continue; //  skip this

      std::string nodeName = "/"+ std::string(node_names.data[i]);

      nlohmann::json json_nodeInfo = get_node_info(nodeName);
      if (!json_nodeInfo.empty()) // add to json_nodeGraph
        json_nodeGraph[nodeName] = json_nodeInfo[nodeName];

    }
    return json_nodeGraph;

  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), e.what());
    return nlohmann::json(); // return an empty string
  }
}



nlohmann::json NODE::get_node_info(const std::string& node_name_){
    // execute cli command to read the node info
      const std::string cmd = "ros2 node info " + std::string(node_name_);
      std::string node_info =  execute_cli_cmd(cmd);

      // add ':' symbol after node name
      size_t pos = node_info.find(node_name_);
      if(pos != std::string::npos){
        node_info.insert(pos + node_name_.size(), ":");
      }

      std::string not_found_err =  "Unable to find node " + node_name_;
      if (node_info == not_found_err) {
        return nlohmann::json();
      }

      // convert string to YAML
      YAML::Node yaml_nodeInfo  = YAML::Load(node_info);
      // convert to json
      nlohmann::json json_nodeInfo = yamlToJson(yaml_nodeInfo);
      return json_nodeInfo;
}



std::string NODE::execute_cli_cmd(const std::string& cmd){
  std::array<char, 128> buffer;
  std::string result;
  const char* cmd_ = cmd.c_str();
  std::shared_ptr<FILE> pipe(popen(cmd_, "r"), pclose);
  if (!pipe) {
    throw std::runtime_error("popen() failed!");
  }
  while (!feof(pipe.get())) {
    if(fgets(buffer.data(), 128, pipe.get()) != nullptr) {
      result += buffer.data();
    }
  }
  return result;
}


nlohmann::json NODE::yamlToJson(const YAML::Node& yaml){
  nlohmann::json result;

  switch (yaml.Type()){
      case YAML::NodeType::Scalar:
          result = yaml.Scalar();
          break;

      case YAML::NodeType::Sequence:
          for(const auto& item : yaml){
              result.push_back(yamlToJson(item));
          }
          break;

      case YAML::NodeType::Map:
          for (const auto& item : yaml){
              result[item.first.Scalar()] = yamlToJson(item.second);
          }
          break;

      default:
            break;
  }

  /// remove nulls if exists;
  for (auto& item : result.items()){
    if (item.value().is_null()){
      item.value() = nlohmann::json::object();
    }

    for (auto& item_: item.value().items()){
      if (item_.value().is_null()){
        item_.value() = nlohmann::json::object();
      }
    }
  }

  return result;
}

std::string NODE::yamlToString(const YAML::Node& yaml){
  std::ostringstream stream;
  YAML::Emitter emitter(stream);
  emitter << yaml;
  return stream.str();
}

YAML::Node timestamp_yaml(){
    YAML::Node ts;

    rcutils_time_point_value_t now_;
    auto ret = rcutils_system_time_now(&now_);
    if(ret != RCL_RET_OK){
        RCUTILS_LOG_ERROR_NAMED("node_name", "Failed to get current time: %s /n", rcutils_get_error_string().str);
        ts["sec"] = 0;
        ts["nanosec"] = 0;
    }
    ts["sec"] = (int32_t)(now_ / 1000000000);
    ts["nanosec"] = (uint32_t)(now_ % 1000000000);
    return ts;
}

void spin(){
  while(true){
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

} // namespace dynrclcpp


