# dynrclcpp
Dynamic ROS 2 client library

This library allows developers to dynamically create and desstroy ROS2 entities such as *publishers, subscriptions, service clients* and *service servers*. 


### Features
- **`Dynamic Creation`**: Programmatically generate publishers, subscriptions, service clients, and service servers on-the-fly.
- **`Flexibility`**: Adapt to changing requirements and environments with ease.
- **`Ease of Use`**: Simple and intuitive API for seamless integration into your ROS2 projects.
- **`Efficiency`**: Optimize resource usage by creating and destroying communication entities as needed,

</br>

### Tested Distro

- [x] Fox
- [x] Galactic
- [x] Humble
- [x] Iron
- [ ] Jezzy

## Instruction:

To create a node with dynamic ROS 2 entities, follow the steps metioned below:

- Install dependencies:
    ```
    sudo apt update
    sudo apt-get install nlohmann-json3-dev
    sudo apt install libyaml-cpp-dev
```
- Clone the `dynrclcpp` library into the src directory of your ROS2 workspace.
    ```
    cd ~/ros2_ws/src
    git clone https://github.com/NDHANA94/dynrclcpp.git
    ```
- Create your ROS2 package with a node:
    ```
    ros2 pkg create <PKG NAME> --node-name <NODE NAME> --dependencies dynmsg dynrclcpp rcl rcl_action ament_index_cpp
    ```
    Create dynamic entities within the Node. Check out [examples](examples/dynrclcpp_examples/src) to understand how to create publishers, subscriptions, service clients and service servers using `dynrclcpp` library.

- build:
    ```
    cd ~/ros2_ws
    colcon build --symlink-install
    ```

## Examples:
- Publishers:
    - [`String` publisher](examples/dynrclcpp_examples/src/string_publisher.cpp)
    - [`Int32` publisher](examples/dynrclcpp_examples/src/int_publisher.cpp)
    - [`int32MultiArray` publisher](examples/dynrclcpp_examples/src/multi_int32_publisher.cpp)
    - [`Point` publisher](examples/dynrclcpp_examples/src/point_publisher.cpp)
    - [`PoseStamped` publisher](examples/dynrclcpp_examples/src/pose_stamped_publisher.cpp)
- Subscriptions:
    - [`String` subscriber](examples/dynrclcpp_examples/src/string_subscriber.cpp)
    - [`Int32` subscriber](examples/dynrclcpp_examples/src/int_subscriber.cpp)
    - [`Float64MultiArray` subscriber](examples/dynrclcpp_examples/src/multi_float64_subscriber.cpp)
- Service Clients:
    - [`AddTwoInts` client](examples/dynrclcpp_examples/src/add_two_ints_client.cpp)
- Service Servers:
    - [`AddTwoInts` server](examples/dynrclcpp_examples/src/add_two_ints_service.cpp)

</br></br>

#### Acknowledgements

This `dynrclcpp` library is developed utilizing `dynmsg` library of [ dynamic_message_introspection](https://github.com/osrf/dynamic_message_introspection).

