
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