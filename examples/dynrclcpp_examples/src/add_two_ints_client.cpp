#include <memory>
#include "dynrclcpp/node_base.hpp"



class AddTwoIntClient{
public:
    AddTwoIntClient(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        node_->init(argc, argv);

        /// initialize the client
        auto callback = std::bind(&AddTwoIntClient::callback_func, this, std::placeholders::_1);
        client_ = node_->create_client("add_two_ints", "example_interfaces/srv/AddTwoInts", QOS_DEFAULT, callback);

        /// request
        YAML::Node req;
        req["a"] = int64_t(10);
        req["b"] = int64_t(20);
        // client_->response_timeout = std::chrono::seconds(5);
        client_->ignore_timeout_ = true;
        client_->send_request(req);

        /// wait 5 seconds and destroy 
        // std::this_thread::sleep_for(std::chrono::seconds(5));
        // client_->destroy();
    }

    void callback_func(YAML::Node res){
        std::cout << res["sum"].as<int64_t>() << std::endl;
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Client> client_;
    int count = 0;

};


int main(int argc, char** argv)
{
    AddTwoIntClient node("int_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}