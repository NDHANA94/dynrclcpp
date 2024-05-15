# dynrclcpp
Dynamic ROS 2 client library


This library empowers developers with the ability to dynamically create and destroy publishers, subscriptions, service clients, and service servers within the ROS2 ecosystem. Whether you're dealing with rapidly changing environments or need fine-gained control over your ROS2 communication channels, this library provides the flexibility and scalability you need.

### Features
- **Dynamic Creation**: Programmatically generate publishers, subscriptions, service clients, and service servers on-the-fly.
- **Flexibility**: Adapt to changing requirements and environments with ease.
- **Ease of Use**: Simple and intuitive API for seamless integration into your ROS2 projects.
- **Efficiency**: Optimize resource usage by creating and destroying communication entities as needed,


## Instruction:

To create a node with dynamic ROS 2 entities, follow the steps metioned below:

- Clone the `dynrclcpp` library into the src directory of your ROS2 package.
    `git clone ***`
- Create your ROS2 package with a node:
    `ros2 pkg create <PKG NAME> --node-name <NODE NAME> --dependencies dynmsg dynrclcpp rcl rcl_action ament_index_cpp`
- Create dynamic entities within the Node. Check out examples to understand how to create publishers, subscriptions, service clients and service servers using `dynrclcpp` library.
- Compile:
    `colcon build --symlink-install`

## Examples:

### 1. Creating dynamic publishers

#### 1.1 publishing a string

```cpp
#include <memory>
#include "dynrclcpp/node_base.hpp"

class StringPublisher{
public:
    StringPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        node_->init(argc, argv);

        // initialize the publisher
        publisher_ = node_->create_publisher("topic", "std_msgs/msg/String", QOS_DEFAULT);

        // bind the publisher to a timer
        if(publisher_ != nullptr)
            publisher_->timer = node_->create_timer("timer1", std::bind(&StringPublisher::timer_callback, this), std::chrono::seconds(1));
    }

    void timer_callback(){
        YAML::Node msg;
        std::string data = "Hello world: " + std::to_string(count++);
        msg["data"] = data;
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "publishing: %s", data.c_str());
        publisher_->publish(msg);
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Publisher> publisher_;
    int count = 0;

};


int main(int argc, char** argv)
{
    StringPublisher node("string_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}
```

</br></br>

#### 1.2 publishing an integer

```cpp
#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntPublisher{
public:
    IntPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
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
```

</br></br>

#### 1.3 publishing an array of integer (Int32MultiArray)

```cpp
#include <iostream>
#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntMultiArrayPublisher{
public:
    IntMultiArrayPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
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
```
</br></br>

#### 1.4 publishing a point


```cpp
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
```
</br></br>


### 2. Creating dynamic subscriptions:

#### 2.1 subscribing to a topic with string type

```cpp
#include <memory>
#include "dynrclcpp/node_base.hpp"

class StringSubscription{
public:
    StringSubscription(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
        node_->init(argc, argv);

        /// initialize the publisher
        auto callback_func = std::bind(&StringSubscription::subscription_callback, this, std::placeholders::_1);

        /// initializing subscriber
        
        subscriber_ = node_->create_subscription("/chatter", QOS_DEFAULT, callback_func);       // Option 1: Without explicitly passing the type of the topic
        // subscriber_ = node_->create_subscription("/chatter", "std_msgs/msg/String", QOS_DEFAULT, callback_func);    // Option 2: With the type of the topic

        // initiate subscription
        if(subscriber_ != nullptr) subscriber_->subscribe();

        // wait 5 seconds
        std::this_thread::sleep_for(std::chrono::seconds(5));

        // destroy subscription
        if(subscriber_ != nullptr) subscriber_->destroy();
        
    }

    void subscription_callback(const YAML::Node msg){
        
        RCUTILS_LOG_INFO_NAMED(node_->node_name.c_str(), "received data: %s", yaml_to_string(msg["data"]).c_str());
    }

private:
    std::shared_ptr<dynrclcpp::NODE> node_;
    std::shared_ptr<dynrclcpp::Subscription> subscriber_;
};


int main(int argc, char** argv)
{
    StringSubscription node("string_publisher", argc, argv);
    dynrclcpp::spin();
    return 0;
}
```

* publish `talker` demo topic to test this: 
    ```
    ros2 run demo_nodes_cpp talker
    ```

</br></br>

#### 2.2 subscibing to a topic with integer type

```cpp
#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntSubscription{
public:
    IntSubscription(const std::string& name, int argc, char** argv){
        /// initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
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
        int data = std::stoi(yaml_to_string(msg["data"]));
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
```
* To test this subscription, create a publisher named `int_topic` :
    ```
    ros2 topic pub /int_topic std_msgs/msg/Int32 "data: 1"
    ```


</br></br>

#### 2.3 publishing an array of integer (Int32MultiArray)

```cpp
#include <iostream>
#include <memory>
#include "dynrclcpp/node_base.hpp"

class IntMultiArrayPublisher{
public:
    IntMultiArrayPublisher(const std::string& name, int argc, char** argv){
        // initialize the node
        node_ = std::make_shared<dynrclcpp::NODE>(name);
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
```
</br></br>

#### 2.4 publishing a point


```cpp
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
```



<!-- 

Done: 
    - Create & destroy Node
    - Create & destroy Publisher
    - Create & destroy Subscription
    - Create & destroy Timer
    - Get nodes Info

TODO:
    - Create & destroy Service client
    - Create & destroy Service server


 -->