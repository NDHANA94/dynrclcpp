
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