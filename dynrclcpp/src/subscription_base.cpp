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

#include "dynrclcpp/subscription_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"

#include "rcl/error_handling.h" 

#include "dynmsg/message_reading.hpp"


namespace dynrclcpp {

Subscription::Subscription(
const std::string& topic_, 
const std::string& type_,
rcl_node_t* node_,
rmw_qos_profile_t qos_,
std::function<void(RosMessage msg)> callback_
)
{
    topic = topic_; 
    type = type_; 
    node = node_; 
    qos  = qos_;
    callback = callback_;


    type_support = get_msg_type_support(type_);
    if(type_support == nullptr) throw std::runtime_error("got a nullptr as type_support");
    sub = rcl_get_zero_initialized_subscription();
    options = rcl_subscription_get_default_options();

    // set qos
    options.qos = qos;
    
    // initialize msg
    interface_type_name = get_interface_type_name_from_type(type);
    if(DYNMSG_RET_OK != dynmsg::c::ros_message_init(interface_type_name, &msg)){
        std::string err = interface_type_name.first + "/" + interface_type_name.second + " msg init failed in topic echo";
        throw std::runtime_error(err.c_str());
    }
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "%s/%s msg was initialized", interface_type_name.first.c_str(), interface_type_name.second.c_str());

    // init subscription
    auto ret_ = rcl_subscription_init(&sub, node, type_support, topic.c_str(), &options);
    if(ret_ != RCL_RET_OK){
        std::string err = "subscription init for topic " + topic +  "failed: " + rcl_get_error_string().str;
        throw std::runtime_error(err);
    }
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "subscription is successfully created");
    is_initialized = true;

    
}



void Subscription::subscribe(){
    if(is_initialized){
        std::thread([this](){
            try{
                count_publishers();
                read_msg(callback);
            }
            catch(const std::runtime_error & e){
                RCUTILS_LOG_ERROR_NAMED(topic.c_str(), e.what());
            }
        }).detach();
    }
}




void Subscription::count_publishers(){
  auto start_time = std::chrono::steady_clock::now();
  while(!stopFlag){
    auto ret = rmw_subscription_count_matched_publishers(rcl_subscription_get_rmw_handle(&sub), &count_pubs);
    if(ret != RCL_RET_OK){
      std::string err = "publisher count for topic " + topic + "failed";
      throw std::runtime_error(err);
    }
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "There are %ld matched publishers", count_pubs);
    if(count_pubs > 0){
      break;
    }
    auto curTime = std::chrono::steady_clock::now();
    auto elapsedTime = std::chrono::duration_cast<std::chrono::seconds>(curTime - start_time);
    if(elapsedTime.count() >= 3) {
      RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "count publisher timeout");
      break;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(250));
  }
}



void Subscription::read_msg(std::function<void(RosMessage msg)> callback_){
    // rmw_take (read msg)
    bool taken = false;
    while (!stopFlag && count_pubs>0){ //.load(std::memory_order_acquire)){
        taken = false;
        auto ret = rmw_take(rcl_subscription_get_rmw_handle(&sub), msg.data, &taken, nullptr);
        if(ret != RCL_RET_OK){
            std::string err = "take failed for subscription " + topic;
            throw std::runtime_error(err.c_str());
        }
        if(taken){
            // invoke callback function
            callback_(msg);
        }
    }

    // clear up
    std::this_thread::sleep_for(std::chrono::milliseconds(200)); 
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "destroying...");
    auto ret = rcl_subscription_fini(&sub, node);
    if(ret != RCL_RET_OK){
        std::string err = "subscription fini for topic " + topic + " failed";
        throw std::runtime_error(err.c_str());
    }
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "subscription successfully destroyed");
}



void Subscription::destroy(){
    stopFlag = true;
}

} //dynrclcpp