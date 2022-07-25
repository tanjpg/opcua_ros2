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

#ifndef OPCUA_SERVER_HPP_
#define OPCUA_SERVER_HPP_

#include <iostream>
#include <algorithm>
#include <time.h>

#include <thread>
#include <chrono>
#include "rclcpp/rclcpp.hpp"

#include <opc/ua/node.h>
#include <opc/ua/subscription.h>
#include <opc/ua/server/server.h>

class EventHandler : public OpcUa::SubscriptionHandler{
  void DataChange(uint32_t handle, const OpcUa::Node & node, const OpcUa::Variant & val, OpcUa::AttributeId attr) override{

    std::cout << node.GetId().GetStringIdentifier() << " has been changed to " << val.ToString() << "\n\n";
  }
};

class OpcUaServer : public OpcUa::UaServer
{
  public:
    OpcUaServer(const std::string endpoint, const std::string server_uri)
    : endpoint_(endpoint),
      server_uri_(server_uri){
        ConnectToServer();
    }

    void ConnectToServer(){
      SetEndpoint(endpoint_);
      SetServerURI(server_uri_);
    }

    void SetupServer(const std::vector<std::string> ns_list){
      server_ns_ = RegisterNamespace(server_uri_);
      for(auto ns : ns_list){
        namespace_indexes_[ns] = RegisterNamespace(ns);
      }
      objects_node_ = GetObjectsNode();
    }

    OpcUa::Node AddNewVariable(
      const std::string node_id,
      const std::string variable_name,
      const OpcUa::Variant & variant,
      const bool read_only){
        OpcUa::Node new_node =
          objects_node_.AddVariable(node_id, variable_name, variant);
        if(read_only){
          new_node.SetAccessLevel(
            OpcUa::VariableAccessLevel::CurrentRead);
        }else{
          new_node.SetAccessLevel(
            OpcUa::VariableAccessLevel::CurrentWrite);
        }
        return new_node;
    }

    uint32_t GetIndex(const std::string ns){
      return namespace_indexes_[ns];
    }
  OpcUa::Node objects_node_;  
  private:
  std::string endpoint_;
  std::string server_uri_;
  std::map<std::string, uint32_t> namespace_indexes_;
  std::map<std::string, std::shared_ptr<OpcUa::Node>> nodes_;

  // std::shared_ptr<OpcUa::UaServer> server;
  uint32_t server_ns_; // Server Namespace
};
#endif  // OPCUA_SERVER_HPP_

