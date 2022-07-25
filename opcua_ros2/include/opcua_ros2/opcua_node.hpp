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

#include <iostream>
#include <algorithm>
#include <time.h>

#include <thread>
#include <chrono>

#include <opc/ua/node.h>
#include <opc/ua/subscription.h>
#include <opc/ua/server/server.h>

using namespace OpcUa;

#ifndef OPCUA_ROS2__OPCUA_NODE_HPP_
#define OPCUA_ROS2__OPCUA_NODE_HPP_

/* Struct definition of a Node. For the current implementation a Node represents 
   a more detailed implementation of the current OpcUa::Node, including old and new
   node values */

namespace opcua_ros2{
  struct Node{
    /**
     * Constructor
     * \param[in] node opcua node instance
     * \param[in] read_only True if node is read only
     * \param[in] tag Node tag
     */
    Node(
      OpcUa::Node node,
      bool read_only,
      std::string tag)
    : node_(node),
      read_only_(read_only),
      tag_(tag){
        curr_value_ = node.GetDataValue();
        old_value_ = node.GetDataValue();
      }
    
    /*! \brief Function that updates value of current node*/
    void UpdateValue(){
      old_value_ = curr_value_;
      curr_value_ = node_.GetValue();
    }

    /*! \brief Attribute of whether a node is read only or not*/
    bool read_only_;

    /*! \brief node's tag name*/
    std::string tag_;

    /*! \brief opcua node instance*/
    OpcUa::Node node_;

    /*! \brief Current value of this node*/
    OpcUa::DataValue curr_value_;

    /*! \brief Old value of this node*/
    OpcUa::DataValue old_value_;
  };


  /* Struct definition of a Component. A component is an object that contains
     multiple Variable Nodes. An example of this could be a conveyor belt with
     start, stop, ready Variable Nodes. */
  
  struct Component{
    /**
     * Constructor
     * \param[in] component_name Currenty component name
     */
    Component(std::string component_name)
    : component_name_(component_name){
      }

    /**
     * Function to add nodes to a component
     * \param[in] node Target opcua node to be added
     * \param[in] read_only True if node is read only
     * \param[in] tag Node tag name
     */
    void AddNode(
      OpcUa::Node node,
      bool read_only,
      std::string tag){
        opcua_nodes_[tag] =
          std::make_shared<opcua_ros2::Node>(
            node,
            read_only,
            tag);
    }

    /*! \brief Current component name*/
    std::string component_name_;

    /*! \brief Map containing OpcUaROS2 nodes mapped to the relevant tag name */
    std::map<std::string, std::shared_ptr<opcua_ros2::Node>> opcua_nodes_;
  };
} // namespace opcua_ros2
#endif  // OPCUA_ROS2__OPCUA_NODE_HPP_
