#include  "dynrclcpp/client_base.hpp"

#include "rcutils/logging.h"
#include "rcutils/logging_macros.h"


namespace dynrclcpp{

Client::Client(
rcl_node_t* node, 
const std::string& name, 
const std::string& type, 
std::function<void(RosMessage msg)> callback)
: node_(node), name_(name), type_str_(type), callback_(std::move(callback))
{
    try{
        client_ = rcl_get_zero_initialized_client();
        type_support_ = get_srv_type_support(type_str_);

        auto ret = rcl_client_init(
            &client_,
            node_,
            type_support_,
            name.c_str(),
            &rcl_client_get_default_options());

        
    }
    catch(const std::runtime_error & e){
        RCUTILS_LOG_ERROR_NAMED(name_.c_str(), "Error while initializing: %s", e.what());
    }

}

void Client::stop(){

}


}
