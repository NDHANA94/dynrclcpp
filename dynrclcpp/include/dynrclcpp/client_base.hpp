#ifndef _DYN_CLIENT_BASE_HPP__
#define _DYN_CLIENT_BASE_HPP__

#include <string>
#include <functional>
#include <thread>
#include <atomic>

#include "rcl/rcl.h"
#include "rcl/error_handling.h"
#include "rcl/client.h"

#include "dynrclcpp/typesupport_utils.hpp"


namespace dynrclcpp{

class Client{
public:
    Client(
    rcl_node_t* node, 
    const std::string& name, 
    const std::string& type, 
    std::function<void(RosMessage msg)> callback);

    void stop();

private:
    std::atomic<bool> stopFlag{false};

    
    rcl_node_t* node_;
    rcl_client_t client_;
    std::string name_;
    std::string type_str_;
    const rosidl_service_type_support_t* type_support_;
    std::function<void(RosMessage msg)> callback_;
    
    
    
};

}


#endif //_DYN_CLIENT_BASE_HPP__