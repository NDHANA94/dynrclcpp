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

#ifndef _DYN_PUBLISHER_BASE_HPP__
#define _DYN_PUBLISHER_BASE_HPP__

#include <memory>

#include "dynrclcpp/typesupport_utils.hpp"
#include "dynrclcpp/timer_base.hpp"
#include "rcl/rcl.h"


namespace dynrclcpp{

class Publisher{
public:
    /// @brief Constructor
    /// @param topic_ : topic name as a string
    /// @param type_ : msg type as a string. Ex: "std_msgs/msg/String"
    /// @param node_ : refernece to the created node
    /// @param qos_ : rmw_qos_profile: 
    Publisher(std::string topic_, 
    std::string type_, 
    rcl_node_t* node_, 
    rmw_qos_profile_t qos_);

    /// @brief Deconstructor
    ~Publisher(){};

    std::string topic;
    std::string type;
    std::shared_ptr<Timer> timer;

    /// @brief Publish the topic
    /// @param msg_yaml : yaml formated message to publish
    void publish(std::string& msg_yaml);

    /// @brief To destroy the publisher
    void destroy();

private:
    InterfaceTypeName interface_type_name;
    const TypeSupport* type_support;
    rcl_publisher_t pub;
    rcl_publisher_options_t options;
    rcl_node_t* node;
    rmw_qos_profile_t qos;
};

} //dynrclcpp

#endif //_DYN_PUBLISHER_BASE_HPP__


    
    

    
    