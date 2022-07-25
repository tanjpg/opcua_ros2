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

#include <thread>
#include <chrono>
#include "opcua_ros2/opcua_ros2_client.hpp"
#include "rclcpp/rclcpp.hpp"
#include <iostream>
#include <signal.h>
#include <stdio.h>
#include <unistd.h>

using namespace std::chrono_literals;
std::chrono::seconds wait_time = 3s;

volatile sig_atomic_t sigint_flag; // Atomic Variable to signal if SIGINT was raised

// Function to trigger atomic sigint_flag
void signal_handler(int signal_num)
{
    sigint_flag = signal_num;
}

// Separate thread function to check and print tag values from server
void check_server_updates(std::shared_ptr<opcua_ros2::Client> client_node){
  while(!sigint_flag){
    client_node->UpdateTagValues();
    std::this_thread::sleep_for(wait_time);
  }
}

// Separate thread function to simulate client changing server values
void trigger_server(std::shared_ptr<opcua_ros2::Client> client_node,
  std::string component_name){
  while(!sigint_flag){
    std::this_thread::sleep_for(wait_time);
    client_node->components_[component_name]->
        opcua_nodes_["start"]->node_.SetValue(true);

    client_node->components_[component_name]->
        opcua_nodes_["stop"]->node_.SetValue(false);

    std::this_thread::sleep_for(wait_time);

    client_node->components_[component_name]->
        opcua_nodes_["stop"]->node_.SetValue(true);
    client_node->components_[component_name]->
        opcua_nodes_["start"]->node_.SetValue(false);

    std::this_thread::sleep_for(wait_time);

    client_node->components_[component_name]->
      opcua_nodes_["reset"]->node_.SetValue(true);

    std::this_thread::sleep_for(wait_time);

    client_node->components_[component_name]->
      opcua_nodes_["reset"]->node_.SetValue(false);
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  // Create instance of ROS2 OpcUa Client
  std::cout << "Creating Client Node" << "\n";
  auto client_node = std::make_shared<opcua_ros2::Client> (
      "client_node",
      node_options
    );

  std::cout << "Loading Components" << "\n";
  client_node->LoadComponents();

  std::cout << "Root node is: " << client_node->opcua_client_->GetRootNode() << "\n";
  std::cout << "Children are: " << "\n";

  for (Node node : client_node->opcua_client_->GetRootNode().GetChildren())
  {
    std::cout << "    {" << node << "}" << "\n";
  }

  auto component_list = client_node->get_parameter("component_list").as_string_array();

  std::vector<std::future<void>> futures;

  // Spawn Thread to constantly print out the value of each Variable Node in the opcua server
  futures.push_back(
    std::async(
      std::launch::async,
      check_server_updates,
      client_node));

  for(auto component : component_list){
    // Spawn Thread to constantly simulate changing Variable node values on the server side
    futures.push_back(
      std::async(
        std::launch::async,
        trigger_server,
        client_node,
        component));
  }

  // // Spawn Thread to constantly simulate changing Variable node values on the server side
  // futures.push_back(
  //   std::async(
  //     std::launch::async,
  //     trigger_server,
  //     client_node,
  //     "machine_1"));

  // // Spawn Thread to constantly simulate changing Variable node values on the server side
  // futures.push_back(
  //   std::async(
  //     std::launch::async,
  //     trigger_server,
  //     client_node,
  //     "machine_2"));

  // Constantly poll to check for SIGINT (Ctrl-C) activated, to gracefully exit program
  try {
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = signal_handler;
    sigaction(SIGINT,&sa,0);
    for(;;){
        pause();
        if (sigint_flag){
          throw (int)sigint_flag;
        }
    }
  }catch(int X){
      printf("SIGINT Caught: %d\n", X);
      return 1;
  }

  rclcpp::shutdown();
  return 0;
}

