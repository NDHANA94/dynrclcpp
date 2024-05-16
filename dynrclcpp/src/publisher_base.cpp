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



#include "dynrclcpp/publisher_base.hpp"

#include "dynmsg/yaml_utils.hpp"
#include "dynmsg/msg_parser.hpp"

#include "rcl/error_handling.h"

#include "rcutils/logging_macros.h"

namespace dynrclcpp{

Publisher::Publisher(std::string topic_, std::string type_, rcl_node_t* node_, rmw_qos_profile_t qos_ = rmw_qos_profile_default):
topic(topic_),  type(type_), node(node_), qos(qos_)
{
    
    type_support = get_msg_type_support(type_);
    pub = rcl_get_zero_initialized_publisher();
    options = rcl_publisher_get_default_options();
    interface_type_name = get_interface_type_name_from_type(type_);

    // set qos
    options.qos = qos;

    // init publisher
    auto ret = rcl_publisher_init(&pub, node, type_support, topic.c_str(), &options);
    if(ret!=RCL_RET_OK){
        std::string err = "failed to init publisher for topic '" + topic + "'. " + std::string(rcl_get_error_string().str);
        throw std::runtime_error(err.c_str());
    }
    RCUTILS_LOG_INFO_NAMED(topic.c_str(), "successfully initialized");
}

void Publisher::publish(const YAML::Node& msg_yaml){
  msg = dynmsg::c::yaml_to_rosmsg(this->interface_type_name, msg_yaml);
  auto ret = rcl_publish(&pub, msg.data, nullptr);
  if(ret!=RCL_RET_OK){
    std::string err = "failed to publish topic '" + this->topic + "'. " + std::string(rcl_get_error_string().str);
    throw std::runtime_error(err);
  }
}

void Publisher::destroy(){
  if(timer != nullptr) timer->stop();
  
  auto ret = rcl_publisher_fini(&pub, node);
  if(ret!=RCL_RET_OK){
    std::string err = "failed to finalize publisher for topic '" + topic + "'. " + std::string(rcl_get_error_string().str);
    throw std::runtime_error(err);
  }
  dynmsg::c::ros_message_destroy(&msg); // ?
  RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "successfully destroyed");
}

} //dynrclcpp