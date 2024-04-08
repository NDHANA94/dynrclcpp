#include  "dynrclcpp/client_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"
#include "rcl/error_handling.h"

#include "dynmsg/typesupport.hpp"
#include "dynmsg/msg_parser.hpp"

namespace dynrclcpp{

Client::Client(
rcl_node_t* node, 
const std::string& name, 
const std::string& type, 
rmw_qos_profile_t qos,
std::function<void(RosSrvResponse msg)> callback)
: node_(node), name_(name), type_str_(type), qos_(qos), callback_(std::move(callback))
{
    
    client_ = rcl_get_zero_initialized_client();
    type_support_ = get_srv_type_support(type_str_);
    auto options = rcl_client_get_default_options();
    options.qos = qos_;
    interface_type_ = get_interface_type_name_from_type(type);

    auto ret = rcl_client_init(
        &client_,
        node_,
        type_support_,
        name.c_str(),
        &options);
    if(ret!=RCL_RET_OK){
        std::string err = "Error initializing client '" + name + "': " + rcl_get_error_string().str;
        throw std::runtime_error(err);
    }
    RCUTILS_LOG_DEBUG_NAMED(name.c_str(), "client is successfully initialized.");

    stopFlag_ = false;
    
}


void Client::send_request(const YAML::Node& yaml_req){
    try{
        std::thread([this, yaml_req](){
        
        RosSrvRequest req = dynmsg::c::yaml_to_request(interface_type_, yaml_req);
        
        RosSrvResponse res;
        auto ret = dynmsg::c::ros_srv_response_init(interface_type_, &res);
        if(ret != DYNMSG_RET_OK){
            throw std::runtime_error("failed to init response");
        }
        
        // repeatedly send request until response is successfully taken or timeout.
        auto startTime = std::chrono::steady_clock::now();
        bool is_taken = false;
        while(!stopFlag_){
            // send request
            ret = rcl_send_request(&client_, req.data, &sequence_number_);
            if(ret != RCL_RET_OK){
                std::string err =  "Failed to send request: " + std::string(rcl_get_error_string().str);
                throw std::runtime_error(err);
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // take response
            rmw_request_id_t response_header;
            ret = rcl_take_response(&client_, &response_header, res.data);
            if(ret != RCL_RET_OK){
                RCUTILS_LOG_DEBUG_NAMED(name_.c_str(), "Waiting for response...");
            } else {
                is_taken = true;
                break;
            }

            if (!ignore_timeout_) {
                auto currentTime = std::chrono::steady_clock::now();
                auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds>(currentTime - startTime);
                if(elapsedTime >= response_timeout){
                    RCUTILS_LOG_WARN_NAMED(name_.c_str(), "response take timeout!");
                    break;
                }   
            }
            
        }
        
        // invoke callback function
        if(is_taken) callback_(res);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // clean up
        dynmsg::c::ros_srv_request_destroy(&req);
        dynmsg::c::ros_srv_response_destroy(&res);
        }).detach();
    }
    catch(const std::runtime_error & e){
        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), e.what());
    }
    
    
}


void Client::destroy(){
    stopFlag_ = true;
    auto ret = rcl_client_fini(&client_, node_);
    if(ret != RCL_RET_OK){
        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), "Failed to finilized");
    }
    RCUTILS_LOG_INFO_NAMED(name_.c_str(), "Successfully finilized");
}


}
