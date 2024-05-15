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

#ifndef _DYN_SUBSCRIPTION_BASE_HPP__
#define _DYN_SUBSCRIPTION_BASE_HPP__

#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include <yaml-cpp/yaml.h>

#include <dynmsg/typesupport.hpp>

#include "dynrclcpp/typesupport_utils.hpp"

#include "rcl/rcl.h"


namespace dynrclcpp {


class Subscription{
public:
    std::string topic;
    std::string type;
    
    /// @brief Constructor 
    /// @throws `std::runtime_error`
    /// @param topic_ : topic name as a string
    /// @param type_ : topic type
    /// @param node_ : pointer to the initialized node
    /// @param qos_ : rmw_qos_profile
    /// @param callback_ :callback function
    Subscription(
    const std::string& topic_, 
    const std::string& type_,
    rcl_node_t* node_,
    rmw_qos_profile_t qos_,
    std::function<void(YAML::Node msg)> callback_);

    

    // Deconstructor
    ~Subscription(){};

    /// @brief  To execute subscription process.
    /// @brief This will run on a separate thread.
    /// @note  Before calling this make sure that the created subscription is not a nullptr.
    /// `if(sub != nullptr) sub->subscribe();
    void subscribe();

    /// This will break the rcl_take while loop and finilize the subscription
    void destroy();

private:
    rcl_subscription_t sub;
    const TypeSupport* type_support;
    InterfaceTypeName interface_type_name;
    rcl_node_t* node;
    rcl_subscription_options_t options;
    RosMessage msg;
    rmw_qos_profile_t qos;
    std::function<void(YAML::Node msg)> callback;
    size_t count_pubs{0};
    bool is_initialized{false};
    std::atomic<bool> stopFlag{false}; // atomic boolean to control subscription loop

    /// @brief count available publishers to the created subscriber and 
    /// update the private member variable `count_pubs`.
    /// @note `timeout = 3 sec.`
    void count_publishers();


    /// @brief To read the topic and invoke the callback function.
    /// \note Run `count_publishers()` function before running this function!!!
    /// @param callback_ 
    void read_msg(std::function<void(YAML::Node msg)> callback_);
    
};


} // dynrclcpp

#endif //_DYN_SUBSCRIPTION_BASE_HPP__