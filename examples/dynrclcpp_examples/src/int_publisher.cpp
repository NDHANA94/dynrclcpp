#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntPublisher{
public:
    IntPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        // initialize the publisher
        publisher_ = node_->create_publisher("topic", "std_msgs/msg/Int32", QOS_DEFAULT);

        // bind the publisher to a timer
        if(publisher_ != nullptr)
            publisher_->timer = node_->create_timer("timer1", std::bind(&IntPublisher::timer_callback, this), std::chrono::seconds(1));
    }

    void timer_callback(){
        YAML::Node msg;
        int data = count++;
        msg["data"] = data;
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "publishing: %i", data);
        publisher_->publish(msg);
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Publisher> publisher_;
    int count = 0;

};


int main(int argc, char** argv)
{
    IntPublisher node("int_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}