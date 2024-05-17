#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntSubscription{
public:
    IntSubscription(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        /// initialize the publisher
        auto callback_func = std::bind(&IntSubscription::subscription_callback, this, std::placeholders::_1);

        /// initializing subscriber
        
        subscriber_ = node_->create_subscription("/int_topic", QOS_DEFAULT, callback_func); // Option 1: Without explicitly passing the type of the topic
        // subscriber_ = node_->create_subscription("/chatter", "std_msgs/msg/Int32", QOS_DEFAULT, callback_func);    // Option 2: With the type of the topic

        // initiate subscription
        if(subscriber_ != nullptr) subscriber_->subscribe();

        // wait 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // destroy subscription
        if(subscriber_ != nullptr) subscriber_->destroy();
        
    }

    void subscription_callback(const YAML::Node msg){
        int data = msg["data"].as<int32_t>();
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "received data: %i", data);
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Subscription> subscriber_;
};


int main(int argc, char** argv)
{
    IntSubscription node("string_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}