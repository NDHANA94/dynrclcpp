#include <memory>
#include "dynrclcpp/node_base.hpp"

class MultiFloatSubscription{
public:
    MultiFloatSubscription(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        node_->init(argc, argv);

        /// initialize the publisher
        auto callback_func = std::bind(&MultiFloatSubscription::subscription_callback, this, std::placeholders::_1);

        /// initializing subscriber
        
        subscriber_ = node_->create_subscription("/multi_float_topic", QOS_DEFAULT, callback_func); // Option 1: Without explicitly passing the type of the topic
        // subscriber_ = node_->create_subscription("/chatter", "std_msgs/msg/Int32", QOS_DEFAULT, callback_func);    // Option 2: With the type of the topic

        // initiate subscription
        if(subscriber_ != nullptr) subscriber_->subscribe();

        // wait 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // destroy subscription
        if(subscriber_ != nullptr) subscriber_->destroy();
        
    }

    void subscription_callback(const YAML::Node msg){
        std::vector<float> data = yaml_to_float_vector(msg);
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "received data:");
        for(const auto& item : data){
          std::cout << "- " << item << std::endl;
        }
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Subscription> subscriber_;
};


int main(int argc, char** argv)
{
    MultiFloatSubscription node("string_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}