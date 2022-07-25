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
#include "opcua_ros2/opcua_ros2_server.hpp"
#include "rclcpp/rclcpp.hpp"

#include <signal.h>
#include <stdio.h>
#include <unistd.h>

using namespace std::chrono_literals;
volatile sig_atomic_t flag; // Atomic Variable to signal if SIGINT was raised

// Function to trigger atomic flag
void signal_handler(int signal_num)
{
    flag = signal_num;
}

// Separate thread function to simulate server tag values changing from server side
void plc_element_start_thread(
  OpcUa::Node &ready_node,
  OpcUa::Node &busy_node){
    printf("Thread Start. \n");
    ready_node.SetValue(false);
    ready_node.SetValue(true);
    printf("Wait. \n");
    busy_node.SetValue(true);
    std::this_thread::sleep_for(5000ms);
    busy_node.SetValue(false);
    ready_node.SetValue(false);
    ready_node.SetValue(true);
    std::this_thread::sleep_for(2000ms);
    printf("Thread Complete. \n");
}

// Separate thread function to simulate server tag values changing from server side
void handle_plc_element_start(
  OpcUa::Node &ready_node,
  OpcUa::Node &busy_node){
    auto start_val = ready_node.GetValue();
    if(start_val){
      plc_element_start_thread(ready_node, busy_node);
    }
}

// Separate thread function to simulate server tag values changing from server side
void start_server(std::shared_ptr<opcua_ros2::Server> server_node, std::string component_name){
  std::vector<std::string> component_list{"system", component_name};
  for(auto component : component_list){
    server_node->components_[component]->opcua_nodes_["ready"]->node_.SetValue(true);
  }
  while(!flag){
    for(auto component : component_list){
      if(component.compare("system") != 0){
        handle_plc_element_start(
          server_node->components_[component]->opcua_nodes_["ready"]->node_,
          server_node->components_[component]->opcua_nodes_["busy"]->node_
        );
      }
    }
  }
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  node_options.allow_undeclared_parameters(true);
  node_options.automatically_declare_parameters_from_overrides(true);

  std::cout << "Creating Server Node" << "\n";
  auto server_node = std::make_shared<opcua_ros2::Server> (
      "server_node",
      node_options
    );

  std::cout << "Starting Server" << "\n";
  server_node->StartServer();

  std::cout << "Loading Components" << "\n";
  server_node->LoadComponents();

  // std::cout << "Root node is: " << server_node->opcua_server_->GetRootNode() << "\n";
  // std::cout << "Children are: " << "\n";

  // for (Node node : server_node->opcua_server_->GetRootNode().GetChildren())
  // {
  //   std::cout << "    {" << node << "}" << "\n";
  // }

  /*
  // Uncomment if you want to simulate random changing of OPCUA nodes from the server side

  std::vector<std::future<void>> futures;
  futures.push_back(
    std::async(
      std::launch::async,
      start_server,
      server_node));
  */

  /* Constantly poll to check for SIGINT (Ctrl-C) activated, to gracefully exit program
     This is exceptionally important for Server side because you need to call Server.close()
     If not the server process continues to run in the background even after program stops*/

  try {
    struct sigaction sa;
    sigemptyset(&sa.sa_mask);
    sa.sa_flags = 0;
    sa.sa_handler = signal_handler;
    sigaction(SIGINT,&sa,0);
    for(;;){
        pause();
        if (flag){
          throw (int)flag;
        }
    }

  }catch(int X){
      printf("SIGINT Caught: %d\n", X);
      return 1;
  }

  rclcpp::shutdown();
  return 0;
}

