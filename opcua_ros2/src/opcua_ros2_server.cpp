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

#include <opcua_ros2/opcua_ros2_server.hpp>

namespace opcua_ros2{

  /**
   * Class Constructor
   * \param[in] node_name ROS2 node name
   * \param[in] node_options Node options
   * \param[in] timeout Timeout for Server
   */

  Server::Server(
    const std::string node_name,
    rclcpp::NodeOptions node_options,
    const uint8_t timeout)
    : Node(node_name, "", node_options){
      opcua_server_ = std::make_shared<OpcUaServer>(
        get_parameter("endpoint").as_string(),
        get_parameter("server_uri").as_string());
      server_started_ = false;
    }

  /**
   * Destructor that stops server if server was started 
   * (Important that this is called when the server is destroyed)
   */

  Server::~Server(){
    if(server_started_){
      std::cout << "Stopping Server. " << "\n";
      opcua_server_->Stop();
      server_started_ = false;
    }
  }

  /**
   * Function that starts OpcUa Server
   */
  void Server::StartServer(){
    opcua_server_->Start();
    server_started_ = true;
  }

  /**
   * Function that loads components found in the config file
   */

  void Server::LoadComponents(){
    auto component_list = get_parameter(
      "component_list").as_string_array();
    opcua_server_->SetupServer(component_list);
    int count = 1;
    for(auto component : component_list){
      LoadComponent(component);
      count++;
    }
  }

  /**
   * Function that loads a particular component 
   * \param[in] component_name target component to load
   */

  void Server::LoadComponent(
    const std::string component_name){
      components_[component_name] = std::make_shared<opcua_ros2::Component>(component_name);
      auto tag_list = get_parameter(
        component_name + ".tag_list").as_string_array();
      std::cout << "Component " << component_name << " contains the following " << tag_list.size() << " tags: " << "\n";
      for (auto tag : tag_list){
        std::cout << tag << "\n";
        auto temp_var_node = opcua_server_->AddNewVariable(
            get_parameter(component_name + ".tags." + tag +".node_id").as_string(),
            get_parameter(component_name + ".tags." + tag +".browse_name").as_string(),
            get_parameter(component_name + ".tags." + tag +".variant").as_bool(),
            get_parameter(component_name + ".tags." + tag +".read_only").as_bool());

        std::shared_ptr<Subscription> sub = opcua_server_->CreateSubscription(100, event_handler_);
        sub->SubscribeDataChange(temp_var_node);
        event_change_handlers_[tag] = sub;
        components_[component_name]->AddNode(
          temp_var_node,
          get_parameter(component_name + ".tags." + tag +".read_only").as_bool(),
          tag);
      }
      std::cout << std::endl;
  }
} // namespace opcua_ros2
