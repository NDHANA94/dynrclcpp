#ifndef _DYNAMIC_NODE_HPP__
#define _DYNAMIC_NODE_HPP__


#include <string>
#include <map>
#include <chrono>
#include <memory>

#include "dynrclcpp/dyn_subscription.hpp"
#include "dynrclcpp/dyn_publisher.hpp"
#include "dynrclcpp/timer_base.hpp"
#include "dynrclcpp/typesupport_utils.hpp"

#include "rcl/rcl.h"
#include "rmw/qos_profiles.h"

namespace dynrclcpp{

class NODE{
public:
    /// @brief Constructor
    /// @param node_name_ : name of the node
    NODE(std::string node_name_):node_name(node_name_){};

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


    void destroy_publisher(const std::string& topic_, const std::string& type_);


    std::shared_ptr<Timer> 
    create_timer(
        const std::string& key, 
        std::function<void()> callback, 
        const std::chrono::duration<long, std::ratio<1l, 1l>>& duration);

    
    void destroy_timer(std::string& key);



    
    void spin();



private:
    int domain_id{0};

    std::string node_name;
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