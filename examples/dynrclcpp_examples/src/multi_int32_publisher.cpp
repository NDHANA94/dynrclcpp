#include <iostream>
#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntMultiArrayPublisher{
public:
    IntMultiArrayPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        // initialize the publisher
        publisher_ = node_->create_publisher("topic", "std_msgs/msg/Int32MultiArray", QOS_DEFAULT);

        // bind the publisher to a timer
        if(publisher_ != nullptr)
            publisher_->timer = node_->create_timer("timer1", std::bind(&IntMultiArrayPublisher::timer_callback, this), std::chrono::seconds(1));
    }

    void timer_callback(){
        YAML::Node msg;
        msg["data"] = dataArray;
        msg["layout"]["data_offset"] = 0;
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "publishing data array: ");
        for(auto& item : dataArray){
          std::cout << item << " ";
        }
        std::cout << std::endl;
        publisher_->publish(msg);
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Publisher> publisher_;
    
    std::vector<int> dataArray = {1, 2, 3, 4, 5};

};


int main(int argc, char** argv)
{
    IntMultiArrayPublisher node("int_multiarr_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}