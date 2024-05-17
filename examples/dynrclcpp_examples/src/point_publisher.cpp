
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

struct Point{
  double x;
  double y;
  double z;
};

class PointPublisher{
public:
    PointPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        // node_->set_debug_severity(RCUTILS_LOG_SEVERITY_DEBUG);
        node_->init(argc, argv);

        // initialize the publisher
        publisher_ = node_->create_publisher("topic", "geometry_msgs/msg/Point", QOS_DEFAULT);

        // bind the publisher to a timer
        if(publisher_ != nullptr)
            publisher_->timer = node_->create_timer("timer1", std::bind(&PointPublisher::timer_callback, this), std::chrono::seconds(1));
    }

    void timer_callback(){
        YAML::Node msg;
        // proceess the point
        point.x = 1.2345;
        point.y = 5.43562;
        point.z = 234.23234;
        // assign the point to msg
        msg["x"] = point.x;
        msg["y"] = point.y;
        msg["z"] = point.z;
        // publish
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "publishing a point: x: %f  y: %f  z: %f", point.x, point.y, point.z);
        publisher_->publish(msg);
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Publisher> publisher_;
    
    Point point;

};


int main(int argc, char** argv)
{
    PointPublisher node("int_multiarr_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}