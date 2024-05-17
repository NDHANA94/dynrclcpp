
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


class AddTwoIntClient{
public:
    AddTwoIntClient(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG); 
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