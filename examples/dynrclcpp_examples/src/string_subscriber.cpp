
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

#include <memory>
#include "dynrclcpp/node_base.hpp"

class StringSubscription{
public:
    StringSubscription(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        /// initialize the publisher
        auto callback_func = std::bind(&StringSubscription::subscription_callback, this, std::placeholders::_1);

        /// initializing subscriber
        /// Option 1: Without explicitly passing the type of the topic
        // subscriber_ = node_->create_subscription("/chatter", "std_msgs/msg/String" QOS_DEFAULT, callback_func);
        /// Option 2: With the type of the topic
        subscriber_ = node_->create_subscription("/chatter", QOS_DEFAULT, callback_func);

        // initiate subscription
        if(subscriber_ != nullptr) subscriber_->subscribe();

        // wait 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // destroy subscription
        if(subscriber_ != nullptr) subscriber_->destroy();
        
    }

    void subscription_callback(const YAML::Node msg){
        std::string data = msg["data"].as<std::string>();
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "received data: %s", data.c_str());
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Subscription> subscriber_;
};


int main(int argc, char** argv)
{
    StringSubscription node("string_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}