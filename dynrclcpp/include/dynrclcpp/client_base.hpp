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

#include "yaml-cpp/yaml.h"

namespace dynrclcpp{

class Client{
public:
    Client(
    rcl_node_t* node, 
    const std::string& name, 
    const std::string& type, 
    rmw_qos_profile_t qos,
    std::function<void(const YAML::Node& request)> callback);

    /// @brief sending a request and taking the response asynchronously.
    /// This will send repeatedly request until response is taken or timeout. 
    /// If the response is not taken within `response_timeout = 5 sec.` time period, this function will be terminated.
    /// you can change the `response_timeout` as you need before calling this function.
    /// If you want to ignore timeout, u can set `ignore_timeout_ = true`.
    /// @param yaml_req request as an instance of YAML::Node 
    void send_request(const YAML::Node& request);

    /// To clean up the created client
    void destroy();
    
    std::chrono::milliseconds response_timeout{5000};
    bool ignore_timeout_{false};

private:
    bool is_initialized{false};
    std::atomic<bool> stopFlag{false};

    
    rcl_node_t* node_;
    rcl_client_t client_;
    std::string name_;
    std::string type_str_;
    rmw_qos_profile_t qos_;
    const rosidl_service_type_support_t* type_support_;
    std::function<void(const YAML::Node& request)> callback_;
    
    
    int64_t sequence_number_;
    InterfaceTypeName interface_type_;
    std::atomic<bool> stopFlag_{false};
    
    
};

}


#endif //_DYN_CLIENT_BASE_HPP__