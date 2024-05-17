#include  "dynrclcpp/client_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"
#include "rcl/error_handling.h"

#include "dynmsg/typesupport.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/message_reading.hpp"

namespace dynrclcpp{

Client::Client(
rcl_node_t* node, 
const std::string& name, 
const std::string& type, 
rmw_qos_profile_t qos,
std::function<void(const YAML::Node& res)> callback)
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
        std::string err = "error initializing client '" + name + "': " + rcl_get_error_string().str;
        throw std::runtime_error(err);
    }
    RCUTILS_LOG_INFO_NAMED(name.c_str(), "client is successfully initialized.");
    is_initialized = true;
    stopFlag_ = false;
    
}


void Client::send_request(const YAML::Node& request){
    if(!is_initialized) return;

    try{
        std::thread([this, request](){
        
        RosSrvRequest req = dynmsg::c::yaml_to_rossrv_req(interface_type_, request);
        
        RosSrvResponse res;
        auto ret = dynmsg::c::rossrv_res_init(interface_type_, &res);
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
                std::string err =  "failed to send request: " + std::string(rcl_get_error_string().str);
                throw std::runtime_error(err);
            }
            RCUTILS_LOG_DEBUG_NAMED(name_.c_str(), "request is sent.");

            // take response
            rmw_request_id_t response_header;
            ret = rcl_take_response(&client_, &response_header, res.data);
            if(ret != RCL_RET_OK){
                RCUTILS_LOG_DEBUG_NAMED(name_.c_str(), "service not available, trying again...");
                std::this_thread::sleep_for(std::chrono::seconds(1));
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
        YAML::Node res_yaml = dynmsg::c::srv_response_to_yaml(res);
        if(is_taken) callback_(res_yaml);

        std::this_thread::sleep_for(std::chrono::milliseconds(200));
        // clean up
        dynmsg::c::rossrv_req_destroy(&req);
        dynmsg::c::rossrv_res_destroy(&res);
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
    }else{
        RCUTILS_LOG_INFO_NAMED(name_.c_str(), "Successfully finilized");
    }
}


}
