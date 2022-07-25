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

#include <opcua_ros2/opcua_ros2_client.hpp>
namespace opcua_ros2{

  /**
   * Class Constructor
   * \param[in] node_name ROS2 node name
   * \param[in] node_options Node options
   * \param[in] timeout Timeout for Server
   */
  Client::Client(
    const std::string node_name,
    rclcpp::NodeOptions node_options,
    const uint8_t timeout)
    : Node(node_name, "", node_options){
      opcua_client_ = std::make_shared<OpcUaClient>(
        get_parameter("endpoint").as_string(),
        get_parameter("server_uri").as_string(),
        get_parameter("client_timeout").as_int());
    }

  /**
   * Function that loads components found in the config file
   */
  void Client::LoadComponents(){
    auto component_list = get_parameter(
        "component_list").as_string_array();
    int count = 1;
    for(auto component : component_list){
      std::cout << "Component" + std::to_string(count) + ")" 
        << component << "\n";
      LoadComponent(component);
      count++;
    }
  }

  /**
   * Function that loads a particular component 
   * \param[in] component_name target component to load
   */
  void Client::LoadComponent(
    const std::string component_name){
      components_[component_name] = std::make_shared<opcua_ros2::Component>(component_name);
      auto tag_list = get_parameter(
        component_name + ".tag_list").as_string_array();
      int count = 1;
      for (auto tag : tag_list){
        std::cout << " \t Tag " + std::to_string(count) + ")" << tag << "\n";
        opcua_client_->GetNode(get_parameter(
          component_name + ".tags." + tag +".node_id").as_string());
        components_[component_name]->AddNode(
          opcua_client_->GetNode(get_parameter(
            component_name + ".tags." + tag +".node_id").as_string()),
          get_parameter(component_name + ".tags." + tag +".read_only").as_bool(),
          tag);
      }
  }

  void Client::UpdateTagValues(){
    std::cout << "Sizes:" << components_.size()<< "\n";
    for(auto &component : components_ ){
      std::cout << "Component:" << component.first<< "\n";
      for(auto &node : component.second->opcua_nodes_){
        node.second->UpdateValue();
        std::cout << "\t" << "Node " << node.first << ": ";
        if(node.second->old_value_.Value.ToString().compare(
          node.second->curr_value_.Value.ToString()) != 0){
            std::cout << "\n" << "\t" << "Old Value:" 
              << node.second->old_value_.Value.ToString() << "\n";
            std::cout << "\t" << "New Value:" 
              << node.second->curr_value_.Value.ToString()<< "\n";
          }else{
            std::cout << node.second->curr_value_.Value.ToString()
              << "(unchanged):" << "\n";
          }
      }
    }
  }
} // namespace opcua_ros2
