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

#ifndef OPCUA_ROS2__OPCUA_ROS2_SERVER_HPP_
#define OPCUA_ROS2__OPCUA_ROS2_SERVER_HPP_


#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>
#include <opc/common/logger.h>

#include <iostream>
#include <stdexcept>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "opcua_ros2/opcua_node.hpp"
#include "opcua_server.hpp"


namespace opcua_ros2{
  class Server : public rclcpp::Node{
    public:
  /**
   * Class Constructor
   * \param[in] node_name ROS2 node name
   * \param[in] node_options Node options
   * \param[in] timeout Timeout for Server
   */
    Server(
      const std::string node_name,
      rclcpp::NodeOptions node_options,
      const uint8_t timeout = 5);

  /**
   * Destructor that stops server if server was started 
   * (Important that this is called when the server is destroyed)
   */
    ~Server();

  /**
   * Function that starts OpcUa Server
   */
    void StartServer();

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

    /*! \brief Map containing system components mapped to the relevant component name */
    std::map<std::string, std::shared_ptr<opcua_ros2::Component>> components_;

    /*! \brief Map containing system change handlers mapped to the relevant tag name */
    std::map<std::string, std::shared_ptr<OpcUa::Subscription>> event_change_handlers_;

    /*! \brief OpcUa Server instance*/
    std::shared_ptr<OpcUaServer> opcua_server_;

    /*! \brief Event handler instance that monitors changes in tag values on the server*/
    EventHandler event_handler_;

    /*! \brief Flag to determine if the server needs to be closed in the destructor*/
    bool server_started_;
  };
} // namespace opcua_ros2

#endif  // OPCUA_ROS2__OPCUA_ROS2_SERVER_HPP_
