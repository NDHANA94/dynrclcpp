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

#ifndef _DYNAMIC_NODE_HPP__
#define _DYNAMIC_NODE_HPP__

#include <string>
#include <map>
#include <chrono>
#include <memory>

#include <yaml-cpp/yaml.h> //sudo apt install libyaml-cpp-dev 
#include <nlohmann/json.hpp> //sudo apt install nlohmann-json3-dev

#include "dynrclcpp/dyn_subscription.hpp"
#include "dynrclcpp/dyn_publisher.hpp"
#include "dynrclcpp/timer_base.hpp"
#include "dynrclcpp/typesupport_utils.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"

#include "rcl/error_handling.h"
#include "rcl/rcl.h"

#include "rmw/qos_profiles.h"

namespace dynrclcpp{

/// @brief This is a ROS2 Node implementation for dynamically and generically create and destroy ROS2 entities such as publishers, subscriptions,
/// service servers, service clients. 
class NODE{
public:
    /// @brief Constructor
    /// @param node_name_ : name of the node
    NODE(const std::string node_name_):node_name(node_name_){};

    /// @brief Deconstructor
    ~NODE(){destroy_node();};

    /// @brief To initiate the node
    /// @param argc x
    /// @param argv 
    void init(int argc, char** argv);

    /// @brief To destroy the node
    void destroy_node();


    /// @brief Change the `domain ID` dynamically.
    /// @brief This will destroy the node and its entities and reinitialize the node.
    /// @param id : domain id
    void change_domain_id(int id);

    /// @brief To set logger serverity
    /// @param log_type : RCUTILS_LOG_SEVERITY_INFO, RCUTILS_LOG_SEVERITY_WARN, 
    /// RCUTILS_LOG_SEVERITY_DEBUG, RCUTILS_LOG_SEVERITY_ERROR, RCUTILS_LOG_SEVERITY_FATAL
    void set_debug_severity(RCUTILS_LOG_SEVERITY log_type);
    

    /// @brief To create a new generic subscription and return shared_ptr of created subscription object.
    /// @param topic_ : topic name as a string
    /// @param type_ : msg type as a string
    /// @param qos_ : rmw_qos_profile
    /// @param callback_ : callback function to be invoked when a msg is received
    /// @return shared_ptr of created subscription object or a nullptr (if failed)
    std::shared_ptr<DynSubscription> create_subscription(
        const std::string& topic_,
        const std::string& type_,
        rmw_qos_profile_t qos_,
        std::function<void(RosMessage msg)> callback_);

    /// @brief To create a new generic subscription without passing type and return shared_ptr of created subscription object.
    /// @param topic_ : topic name as a string
    /// @param type_ : msg type as a string
    /// @param qos_ : rmw_qos_profile
    /// @param callback_ : callback function to be invoked when a msg is received
    /// @return shared_ptr of created subscription object or a nullptr (if failed)
    std::shared_ptr<DynSubscription> create_subscription(
        const std::string& topic_,
        rmw_qos_profile_t qos_,
        std::function<void(RosMessage msg)> callback_);


    /// @brief Destroy a created subscription
    /// @param topic_ : topic name as a string
    /// @param type_ : msg type as a string
    void destroy_subscription(const std::string& topic_, const std::string& type_);

    /// @brief 
    /// @param topic 
    /// @param type 
    std::shared_ptr<DynPublisher> create_publisher(
        const std::string& topic,
        const std::string& type,
        rmw_qos_profile_t qos_);


    /// @brief To dynamically destroy a created publisher
    /// @param topic_ : topic name as a string
    /// @param type_ : type as a string
    void destroy_publisher(const std::string& topic_, const std::string& type_);


    /// @brief Create a timer to execute a task indipendently with a given period
    /// @tparam Rep 
    /// @tparam Period 
    /// @param key a key as a string to store the timer to use when we want to destroy it dynamically
    /// @param callback callback function to invoke
    /// @param duration std::chrono::microseconds(x) or std::chrono::milliseconds(x) or std::chrono::seconds(x)
    /// @return shared ptr of created timer instance
    /// @bug can't define this function in cpp file, linker can't find it
    template <typename Rep, typename Period>
    std::shared_ptr<Timer> 
    create_timer(
        const std::string& key, 
        std::function<void()> callback, 
        const std::chrono::duration<Rep, Period>& duration)
    {
        RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "creating a timer '%s'", key.c_str());
        try{
            if(timer_registry.find(key) != timer_registry.end()){
                throw std::runtime_error("timer with given key alread exists");
            }
            auto timer = Timer::create(std::move(callback), duration);
            timer_registry.insert({key, timer});
            if(timer_registry.find(key) != timer_registry.end())
                RCUTILS_LOG_INFO_NAMED(node_name.c_str(), "added timer: %s", key.c_str());
            return timer;
        } 
        catch(const std::runtime_error & e){
            RCUTILS_LOG_ERROR_NAMED(node_name.c_str(), "Error while creating timer '%s': %s", key.c_str(),  e.what());
            return nullptr;
        }
    }


    /// @brief To destroy a created timer dynamically
    /// @param key the keyword which is used to create the timer
    void destroy_timer(const std::string& key);


    /// @brief to get the ROS graph as a json object
    /// @return nlohmann::json object
    nlohmann::json get_nodes_info();
    nlohmann::json get_node_info(const std::string& node_name_);
    



    void spin();


    const std::string node_name;
private:
    /// @brief to execute cli command 
    /// @param cmd : command to execute as a string
    /// @return result of the executed command as a string
    std::string execute_cli_cmd(const std::string& cmd);
    nlohmann::json yamlToJson(const YAML::Node& yaml);
    std::string yamlToString(const YAML::Node& yaml);


    int domain_id{0};

    rcl_node_t node;

    rcl_init_options_t init_option;
    rcl_node_options_t node_options;
    rcl_allocator_t allocator;
    rcl_context_t context;

    bool shutdown{false};
   
    //    map< pair< topic, type >, entity >
    std::map<std::pair<std::string, std::string>, std::shared_ptr<DynSubscription>> sub_registry;
    //    map< pair< topic, type >, entity >
    std::map<std::pair<std::string, std::string>, std::shared_ptr<DynPublisher>> pub_registry;
    //  map<key, Timer>
    std::map<std::string, std::shared_ptr<Timer>> timer_registry;

};

}

#endif //_DYNAMIC_NODE_HPP__