


#include "dynrclcpp/dyn_publisher.hpp"

#include "dynmsg/yaml_utils.hpp"
#include "dynmsg/msg_parser.hpp"

#include "rcl/error_handling.h"

#include "rcutils/logging_macros.h"



DynPublisher::DynPublisher(std::string topic_, std::string type_, rcl_node_t* node_, rmw_qos_profile_t qos_ = rmw_qos_profile_default):
topic(topic_),  type(type_), node(node_), qos(qos_)
{
    interface_type_name = get_topic_type_from_string_type(type_);
    type_support = get_type_support(interface_type_name);
    pub = rcl_get_zero_initialized_publisher();
    options = rcl_publisher_get_default_options();

    // set qos
    options.qos = qos;

    // init publisher
    auto ret = rcl_publisher_init(&pub, node, type_support, topic.c_str(), &options);
    if(ret!=RCL_RET_OK){
        std::string err = "failed to init publisher for topic '" + topic + "'. " + std::string(rcl_get_error_string().str);
        throw std::runtime_error(err.c_str());
    }
    RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "successfully initialized");
}

void DynPublisher::publish(std::string& msg_yaml){
  RosMessage msg = dynmsg::c::yaml_to_rosmsg(this->interface_type_name, msg_yaml);
  auto ret = rcl_publish(&pub, msg.data, nullptr);
  if(ret!=RCL_RET_OK){
    std::string err = "failed to publish topic '" + this->topic + "'. " + std::string(rcl_get_error_string().str);
    throw std::runtime_error(err);
  }
  RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "published: %s", msg_yaml.c_str());
}

void DynPublisher::destroy(){
  if(timer != nullptr) timer->stop();
  
  auto ret = rcl_publisher_fini(&pub, node);
  if(ret!=RCL_RET_OK){
    std::string err = "failed to finalize publisher for topic '" + topic + "'. " + std::string(rcl_get_error_string().str);
    throw std::runtime_error(err);
  }
  RCUTILS_LOG_DEBUG_NAMED(topic.c_str(), "successfully destroyed");
}
