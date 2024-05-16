#include <memory>
#include "dynrclcpp/node_base.hpp"



class AddTwoIntsService{
public:
    AddTwoIntsService(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        // initialize the client
        auto callback = std::bind(&AddTwoIntsService::callback_func, this, std::placeholders::_1, std::placeholders::_2);
        service_ = node_->create_service("add_two_ints", "example_interfaces/srv/AddTwoInts", QOS_DEFAULT, callback);
    }

    void callback_func(const YAML::Node req, YAML::Node& res){
        int64_t a = req["a"].as<int64_t>();
        int64_t b = req["b"].as<int64_t>();
        res["sum"] = a+b;
        // std::cout << res["sum"] << std::endl;
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Service> service_;
    int count = 0;

};


int main(int argc, char** argv)
{
    AddTwoIntsService node("int_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}