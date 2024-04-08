// Copyright 2020 Open Source Robotics Foundation, Inc.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include <dlfcn.h>

#include <sstream>
#include <string>

#include "dynrclcpp/typesupport_utils.hpp"

#include "rcl/error_handling.h"
#include "rcl/graph.h"
#include "rcutils/logging_macros.h"

std::string get_topic_type_string(const rcl_node_t * node, const std::string & topic)
{
  auto pubs = rcl_get_zero_initialized_topic_endpoint_info_array();
  auto allocator = rcl_get_default_allocator();
  // Find a publisher for the requested topic
  auto ret = rcl_get_publishers_info_by_topic(node, &allocator, topic.data(), false, &pubs);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(rcl_get_error_string().str);
  }
  if (pubs.size == 0) {
    throw std::runtime_error("unable to determine topic type");
  }
  // Get the topic type from the graph information
  std::string topic_type(pubs.info_array->topic_type);
  std::string pkg = topic_type.substr(0, topic_type.find('/'));
  topic_type.insert(pkg.size(), "/msg");
  // std::string name = topic_type.substr(topic_type.rfind('/') + 1);
  // InterfaceTypeName int_type_name{pkg, name};

  // Clean up
  ret = rcl_topic_endpoint_info_array_fini(&pubs, &allocator);
  if (ret != RCL_RET_OK) {
    throw std::runtime_error(rcl_get_error_string().str);
  }
  return topic_type;
}


InterfaceTypeName get_interface_type_name_from_type(const std::string & type)
{
  // std::string::size_type split_at = type.find('/');
  // if (split_at == std::string::npos) {
  //   throw std::runtime_error("invalid type specification");
  // }
  // return InterfaceTypeName(type.substr(0, split_at), type.substr(split_at + 1));

  std::string pkg = type.substr(0, type.find('/'));
  std::string name = type.substr(type.rfind('/') + 1);
  InterfaceTypeName interface{pkg, name};
  if(interface.first == "" || interface.second == ""){
    std::string err = "Unknown topic type: " + type;
    throw std::runtime_error(err.c_str());
  }

  return interface;
}

const rosidl_message_type_support_t * get_msg_type_support(const std::string& type)
{
  std::string pkg_name = type.substr(0, type.find('/'));            //first part
  std::string category_name = type.substr(pkg_name.size() + 1, 3);  // second part
  std::string type_name = type.substr(type.rfind('/') + 1);         // third part
  if(pkg_name == "" || category_name == "" || type_name == ""){
    std::string err = "Unknown topic type: " + type;
    throw std::runtime_error(err.c_str());
  }
  if(category_name != "msg"){
    std::string err = "Invalid topic type: " + type;
    throw std::runtime_error(err.c_str());
  }
  // Load the type support library for the package containing the requested type
  std::string ts_lib_name;
  ts_lib_name = "lib" + pkg_name + "__rosidl_typesupport_c.so";

  RCUTILS_LOG_DEBUG_NAMED("typesupport_utils", "Loading type support library %s", ts_lib_name.c_str());

  void * type_support_lib = dlopen(ts_lib_name.c_str(), RTLD_LAZY);
  if (type_support_lib == nullptr) {
    RCUTILS_LOG_ERROR_NAMED("typesupport_utils", "failed to load type support library: %s", dlerror());
    std::string err = "failed to load type support library for type " + type;
    throw std::runtime_error(err.c_str());
  }
  // Load the function that, when called, will give us the type support for the interface type we
  // are interested in
  std::string ts_func_name;
  ts_func_name = "rosidl_typesupport_c__get_message_type_support_handle__" + pkg_name +
    "__msg__" + type_name;
  RCUTILS_LOG_DEBUG_NAMED("typesupport_utils", "Loading type support function %s", ts_func_name.c_str());

  get_message_ts_func type_support_handle_func =
    reinterpret_cast<get_message_ts_func>(dlsym(type_support_lib, ts_func_name.c_str()));
  if (type_support_handle_func == nullptr) {
    RCUTILS_LOG_ERROR_NAMED("typesupport_utils", "failed to load type support function: %s", dlerror());
    std::string err = "failed to load type support function " + type;
    throw std::runtime_error(err.c_str());
  }

  // Call the function to get the type support we want
  const rosidl_message_type_support_t * ts = type_support_handle_func();
  RCUTILS_LOG_DEBUG_NAMED("typesupport_utils", "Loaded type support %s", ts->typesupport_identifier);

  return ts;
}

const rosidl_service_type_support_t * get_srv_type_support(const std::string& type)
{
  std::string pkg_name = type.substr(0, type.find('/'));            //first part
  std::string category_name = type.substr(pkg_name.size() + 1, 3);  // second part
  std::string type_name = type.substr(type.rfind('/') + 1);         // third part
  if(pkg_name == "" || category_name == "" || type_name == ""){
    std::string err = "Unknown topic type: " + type;
    throw std::runtime_error(err.c_str());
  }
  if(category_name != "srv"){
    std::string err = "Invalid srv type: " + type;
    throw std::runtime_error(err.c_str());
  }

  // Load the type support library for the package containing the requested type
  std::string ts_lib_name;
  ts_lib_name = "lib" + pkg_name + "__rosidl_typesupport_c.so";
  RCUTILS_LOG_DEBUG_NAMED(
    "typesupport_utils",
    "Loading type support library %s",
    ts_lib_name.c_str());
  void * type_support_lib = dlopen(ts_lib_name.c_str(), RTLD_LAZY);
  if (type_support_lib == nullptr) {
    RCUTILS_LOG_ERROR_NAMED("typesupport_utils", "failed to load type support library: %s", dlerror());
    std::string err = "failed to load type support library for type " + type;
    throw std::runtime_error(err.c_str());
  }

  // Load the function that, when called, will give us the type support for the interface type we
  // are interested in
  std::string ts_func_name;
  ts_func_name = "rosidl_typesupport_c__get_service_type_support_handle__" + pkg_name +
    "__srv__" + type_name;
  RCUTILS_LOG_DEBUG_NAMED("typesupport_utils", "Loading type support function %s", ts_func_name.c_str());

  get_service_ts_func type_support_handle_func =
    reinterpret_cast<get_service_ts_func>(dlsym(type_support_lib, ts_func_name.c_str()));
  if (type_support_handle_func == nullptr) {
    RCUTILS_LOG_ERROR_NAMED("typesupport_utils", "failed to load type support function: %s", dlerror());
    std::string err = "failed to load type support function " + type;
    throw std::runtime_error(err.c_str());
  }

  // Call the function to get the type support we want
  const rosidl_service_type_support_t * ts = type_support_handle_func();
  RCUTILS_LOG_DEBUG_NAMED("typesupport_utils", "Loaded type support %s", ts->typesupport_identifier);

  return ts;
}