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

#ifndef OPCUA_CLIENT_HPP_
#define OPCUA_CLIENT_HPP_


#include <opc/ua/client/client.h>
#include <opc/ua/node.h>
#include <opc/ua/subscription.h>
#include <opc/common/logger.h>

#include <iostream>
#include <stdexcept>
#include <thread>

using namespace OpcUa;

class OpcUaClient : public OpcUa::UaClient
{
  public:
    OpcUaClient(
      const std::string endpoint,
      const std::string server_uri,
      const uint8_t timeout = 5)
    : endpoint_(endpoint),
      server_uri_(server_uri){
        DefaultTimeout = timeout;
        try{
          std::cout << "Connecting to: " << endpoint << "\n";
          Connect(endpoint_);
          root_node_ = GetRootNode();
          objects_node_ = GetObjectsNode();
        }
        catch (const std::exception & exc){
          std::cout << "Error: " << exc.what() << "\n";
        }
        catch (...){
          std::cout << "Unknown error." << "\n";
        }
      }

    OpcUaClient(
      const std::string endpoint,
      const std::string server_uri,
      const std::string username,
      const std::string password,
      const uint8_t timeout = 5)
    : endpoint_(endpoint),
      server_uri_(server_uri){
        DefaultTimeout = timeout;
        try{
          std::cout << "Connecting to: " << endpoint << "\n";
          Connect(endpoint_);
          // Password Auth not supported yet
        }
        catch (const std::exception & exc){
          std::cout << "Error: " << exc.what() << "\n";
        }
        catch (...){
          std::cout << "Unknown error." << "\n";
        }
      }
  // private:
  OpcUa::Node root_node_;
  OpcUa::Node objects_node_;
  std::string endpoint_;
  std::string server_uri_;
  uint32_t server_ns_; // Server Namespace
};
#endif  // OPCUA_CLIENT_HPP_
