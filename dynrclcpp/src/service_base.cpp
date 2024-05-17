
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

#include  "dynrclcpp/service_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"
#include "rcl/error_handling.h"

#include "dynmsg/typesupport.hpp"
#include "dynmsg/msg_parser.hpp"
#include "dynmsg/message_reading.hpp"

namespace dynrclcpp
{

Service::Service(
rcl_node_t* node, 
const std::string& name, 
const std::string& type, 
rmw_qos_profile_t qos,
std::function<void(const YAML::Node& req, YAML::Node& res)> callback)
: node_(node), name_(name), type_str_(type), qos_(qos), callback_(std::move(callback))
{

    try{
        stopFlag_ = false;
        // initialize the service
        service_ = rcl_get_zero_initialized_service();
        type_support_ = get_srv_type_support(type_str_);
        auto options = rcl_service_get_default_options();
        options.qos = qos_;
        interface_type_ = get_interface_type_name_from_type(type_str_);

        auto ret = rcl_service_init(
            &service_,
            node_,
            type_support_,
            name_.c_str(),
            &options
        );

        if(ret != RCL_RET_OK){
            std::string err = "Failed to initialize service `" + name_ + "`: " + rcl_get_error_string().str;
            throw std::runtime_error(err);
        }
        // RCUTILS_LOG_INFO_NAMED(name_.c_str(), "service is successfully initialized.");
        // is_initialized = true;

        std::thread([this](){
            // initialize service request
            RosSrvRequest req;
            auto ret = dynmsg::c::rossrv_req_init(interface_type_, &req);
            if(ret != DYNMSG_RET_OK){
                throw std::runtime_error("failed to init request");
            }

            // take request and send response
            while(!stopFlag_){
                rmw_request_id_t request_header;
                ret = rcl_take_request(&service_, &request_header, req.data);
                if(ret != RCL_RET_OK){
                    RCUTILS_LOG_DEBUG_NAMED(name_.c_str(), "Waiting for request...");
                }
                else {
                    YAML::Node res_yaml;
                    YAML::Node req_yaml = dynmsg::c::srv_request_to_yaml(req);
                    //  invoke callback function  
                    callback_(req_yaml, res_yaml);
                    // send response
                    RosSrvResponse res = dynmsg::c::yaml_to_rossrv_res(interface_type_, res_yaml);
                    ret = rcl_send_response(&service_, &request_header, res.data);
                    if(ret != RCL_RET_OK){
                        std::string err = "failed to send response: "  + std::string(rcl_get_error_string().str);
                        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), err.c_str());
                    } else{
                        RCUTILS_LOG_DEBUG_NAMED(name_.c_str(), "Response is sent.");
                    }
                    // clean up response instance
                    dynmsg::c::rossrv_res_destroy(&res);
                }
            }
            // clean up
            dynmsg::c::rossrv_req_destroy(&req);
        }).detach();

        RCUTILS_LOG_INFO_NAMED(name_.c_str(), "service is successfully initialized.");
    }
    catch(const std::runtime_error & e){
        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), e.what());
    }

}


void Service::destroy(){
    stopFlag_ = true;
    auto ret = rcl_service_fini(&service_, node_);
    if(ret != RCL_RET_OK){
        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), "Failed to finilized");
    } else{
        RCUTILS_LOG_INFO_NAMED(name_.c_str(), "Successfully finilized");
    }
}


} // namespace dynrclcpp
