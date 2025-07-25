// Copyright 2019 Open Source Robotics Foundation, Inc.
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

#include <algorithm>
#include <memory>
#include <string>
#include <vector>

#include "class_loader/class_loader.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/node_factory.hpp"
#include "rclcpp_components/node_factory_template.hpp"

#define NODE_MAIN_LOGGER_NAME "task_server_ik_node"

using namespace rclcpp::executors;
using namespace rclcpp::experimental::executors;

int main(int argc, char * argv[])
{
  auto args = rclcpp::init_and_remove_ros_arguments(argc, argv);
  rclcpp::Logger logger = rclcpp::get_logger(NODE_MAIN_LOGGER_NAME);
  SingleThreadedExecutor exec;
  rclcpp::NodeOptions options;
  options.arguments(args);

  std::string library_name = "libtask_server_ik.so";
  std::string class_name = "rclcpp_components::NodeFactoryTemplate<arduinobot_remote_ik::TaskServerIK>";

  RCLCPP_DEBUG(logger, "Load library %s", library_name.c_str());
  auto loader = std::make_unique<class_loader::ClassLoader>(library_name);
  std::vector<std::string> classes = loader->getAvailableClasses<rclcpp_components::NodeFactory>();

  if (std::find(
    classes.begin(),
    classes.end(),
    class_name) == classes.end()) {
    RCLCPP_ERROR(
      logger,
      "Class %s not found in library %s",
      class_name.c_str(),
      library_name.c_str());
    return 1;
  }
  RCLCPP_DEBUG(logger, "Instantiate class %s", class_name.c_str());
  std::shared_ptr<rclcpp_components::NodeFactory> node_factory = nullptr;
  try {
    node_factory = loader->createInstance<rclcpp_components::NodeFactory>(class_name);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(logger, "Failed to load library %s", ex.what());
    return 1;
  } catch (...) {
    RCLCPP_ERROR(logger, "Failed to load library");
    return 1;
  }
  // Scope to destruct node_wrapper before shutdown
  {
    rclcpp_components::NodeInstanceWrapper node_wrapper = node_factory->create_node_instance(options);
    rclcpp::node_interfaces::NodeBaseInterface::SharedPtr node = node_wrapper.get_node_base_interface();
    exec.add_node(node);

    exec.spin();

    exec.remove_node(node_wrapper.get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
