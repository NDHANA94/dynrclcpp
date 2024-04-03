#ifndef _DYN_SUBSCRIPTION_HPP__
#define _DYN_SUBSCRIPTION_HPP__

#include <string>
#include <thread>
#include <atomic>
#include <functional>

#include <dynmsg/typesupport.hpp>

#include "dynrclcpp/typesupport_utils.hpp"

#include "rcl/rcl.h"

class DynSubscription{
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
    DynSubscription(
    const std::string& topic_, 
    const std::string& type_,
    rcl_node_t* node_,
    rmw_qos_profile_t qos_,
    std::function<void(RosMessage msg)> callback_);

    

    // Deconstructor
    ~DynSubscription(){};

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
    std::function<void(RosMessage msg)> callback;
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
    void read_msg(std::function<void(RosMessage msg)> callback_);
    
};

#endif //_DYN_SUBSCRIPTION_HPP__