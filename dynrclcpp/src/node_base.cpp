
#include "dynrclcpp/node_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"

#include "rcl/error_handling.h"
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




std::shared_ptr<DynSubscription> NODE::create_subscription(
const std::string& topic_,
const std::string& type_,
rmw_qos_profile_t qos_,
std::function<void(RosMessage msg)> callback_)
{
  
  // need to sleep for a bit for discovery to populate the ROS graph information 
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try{

    // check if the subscription is already available
    auto it = sub_registry.find({topic_, type_});
    if(it != sub_registry.end()){
      throw std::runtime_error("subscription already exists.");
    }

    // create new subscription
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "creating subscription: '%s' ", topic_.c_str());
    std::shared_ptr<DynSubscription> sub 
      = std::make_shared<DynSubscription>(topic_, type_, &node, qos_, callback_);
    
    // insert created subscription into subs_registry
    sub_registry.insert({{topic_, type_}, sub});
    RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "sub is added to subs_registry as key {%s, %s}", topic_.c_str(), type_.c_str());
    return sub;
  } 
  
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating subscription '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  
}



std::shared_ptr<DynSubscription> NODE::create_subscription(
const std::string& topic_,
rmw_qos_profile_t qos_,
std::function<void(RosMessage msg)> callback_)
{
  
  // need to sleep for a bit for discovery to populate the ROS graph information 
  std::this_thread::sleep_for(std::chrono::seconds(1));
  try{

    auto interface_type_name = get_topic_type(&node, topic_);
    std::string type = interface_type_name.first + "/msg/" + interface_type_name.second;
    // check if the subscription is already available
    auto it = sub_registry.find({topic_, type});
    if(it != sub_registry.end()){
      throw std::runtime_error("subscription already exists.");
    }

    // create new subscription
    RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "creating subscription: '%s' ", topic_.c_str());
    std::shared_ptr<DynSubscription> sub 
      = std::make_shared<DynSubscription>(topic_, type, &node, qos_, callback_);
    
    // insert created subscription into subs_registry
    sub_registry.insert({{topic_, type}, sub});
    RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "sub is added to subs_registry as key {%s, %s}", topic_.c_str(), type.c_str());
    return sub;
  } 
  
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating subscription '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  
}



void NODE::destroy_subscription(const std::string& topic_, const std::string& type_)
{
  RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "Destroying subscription '%s' | %s", topic_.c_str(), type_.c_str());
  try{
    // destroy the subscriber
    auto it = sub_registry.find(std::make_pair(topic_, type_));
    if(it != sub_registry.end()){
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "destroying subscription...");
      auto sub_to_fini = it->second;
      sub_registry.erase({topic_, type_});
      sub_to_fini->destroy();
      
    }
    else{
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "subscription is not found to destroy");
    }
      
  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error: %s: %s", topic_.c_str(), e.what());
  }
  
}



std::shared_ptr<DynPublisher> NODE::create_publisher(
const std::string& topic_,
const std::string& type_,
rmw_qos_profile_t qos_)
{
  try{
    std::shared_ptr<DynPublisher> pub = std::make_shared<DynPublisher>(topic_, type_, &node, qos_);
    pub_registry.insert({{topic_, type_}, pub});
    return pub;
  }
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating publisher '%s': %s", topic_.c_str(),  e.what());
    return nullptr;
  }
  

}

void NODE::destroy_publisher(const std::string& topic_, const std::string& type_){
  RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "Destroying publisher '%s' | %s", topic_.c_str(), type_.c_str());
  try{
    // destroy the subscriber
    auto it = pub_registry.find(std::make_pair(topic_, type_));
    if(it != pub_registry.end()){
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "destroying subscription...");
      auto sub_to_fini = it->second;
      pub_registry.erase({topic_, type_});
      sub_to_fini->destroy();
      
    }
    else{
      RCUTILS_LOG_DEBUG_NAMED(node_name.c_str(), "publisher is not found to destroy");
    }
      
  } catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error: %s: %s", topic_.c_str(), e.what());
  }
}



std::shared_ptr<Timer> 
NODE::create_timer(
const std::string& key, 
std::function<void()> callback, 
const std::chrono::duration<long, std::ratio<1l, 1l>>& duration){
  try{
    if(timer_registry.find(key) != timer_registry.end()){
      throw std::runtime_error("timer with given key alread exists");
    }
    auto timer = Timer::create(std::move(callback), duration);
    timer_registry.insert({key, timer});
    return timer;
  } 
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating timer '%s': %s", key.c_str(),  e.what());
    return nullptr;
  }
}


void NODE::destroy_timer(std::string& key){
  try{
    if(timer_registry.find(key) != timer_registry.end()){
      std::string err = "timer '" + key + "' not found";
      throw std::runtime_error(err);
    }
    timer_registry.erase({key});
    
  } 
  catch(const std::runtime_error & e){
    RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating timer '%s': %s", key.c_str(),  e.what());
  }
}

void NODE::spin(){
  while(!shutdown){
    std::this_thread::sleep_for(std::chrono::seconds(1));
  }
}

} // namespace dynrclcpp

