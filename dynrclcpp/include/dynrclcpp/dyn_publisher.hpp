#ifndef _DYN_PUBLISHER_HPP__
#define _DYN_PUBLISHER_HPP__

#include "dynrclcpp/typesupport_utils.hpp"
#include "rcl/rcl.h"

class DynPublisher{
public:
    /// @brief Constructor
    /// @param topic_ : topic name as a string
    /// @param type_ : msg type as a string. Ex: "std_msgs/msg/String"
    /// @param node_ : refernece to the created node
    /// @param qos_ : rmw_qos_profile: 
    DynPublisher(std::string topic_, 
    std::string type_, 
    rcl_node_t* node_, 
    rmw_qos_profile_t qos_);

    /// @brief Deconstructor
    ~DynPublisher(){};

    std::string topic;
    std::string type;

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

#endif //_DYN_PUBLISHER_HPP__


    
    

    
    