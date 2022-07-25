// Copyright 2022 Advanced Remanufacturing and Technology Centre
// Copyright 2022 ROS-Industrial Consortium Asia Pacific Team
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

#ifndef OPCUA_ROS2__OPCUA_ROS2_CLIENT_HPP_
#define OPCUA_ROS2__OPCUA_ROS2_CLIENT_HPP_

#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>
#include <opc/common/logger.h>

#include <iostream>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "opcua_ros2/opcua_node.hpp"
#include "opcua_client.hpp"

namespace opcua_ros2{
  class Client : public rclcpp::Node{
    public:
  /**
   * Class Constructor
   * \param[in] node_name ROS2 node name
   * \param[in] node_options Node options
   * \param[in] timeout Timeout for Client
   */
      Client(
        const std::string node_name,
        rclcpp::NodeOptions node_options,
        const uint8_t timeout = 5);

  /**
   * Destructor
   */
      ~Client(){}

  /**
   * Function that loads components found in the config file
   */
      void LoadComponents();

  /**
   * Function that loads a particular component 
   * \param[in] component_name target component to load
   */
      void LoadComponent(
        const std::string component_name);

  /**
   * Function that prints out all node values in the server on the terminal.
   */
      void UpdateTagValues();

      /*! \brief Map containing system components mapped to the relevant component name */
      std::map<std::string, std::shared_ptr<opcua_ros2::Component>> components_;

      /*! \brief OpcUa Client instance*/
      std::shared_ptr<OpcUaClient> opcua_client_;
  };
} // namespace opcua_ros2
#endif  // OPCUA_ROS2__OPCUA_ROS2_CLIENT_HPP_
