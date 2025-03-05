/*
Copyright (c) 2020, Marco Faroni
Poitecnico di Milano
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#pragma once

#include <cstdio>
#include <rclcpp/rclcpp.hpp>
#include <Eigen/Dense>

#include "ssm_safety_ros/ssm_ros_base_node_library.h"
#include <velocity_scaling_iso15066/ssm_fixed_areas.h>

// first we define a struct containing the fixed area's info
struct Area
{
  // TODO: this is ok if we only have two shapes, should use polimorphism
  std::string name;
  double override;
  std::vector<std::vector<double>> corners;
  double radius;
};

// second we specialize a template function from cnr_param to use custom structs
namespace YAML
{
template <>
struct convert<Area>
{
  static Node encode(const Area& rhs)
  {
    Node node;
    node["name"] = rhs.name;
    node["override"] = rhs.override;
    if (rhs.corners.size()>0)
    {
      node["corners"] = rhs.corners;
    }
    else
    {
      node["radius"] = rhs.radius;
    }
    return node;
  }

  static bool decode(const Node& node, Area& rhs)
  {
    if (!node.IsMap() || !node["name"] || !node["override"])
    {
      return false;
    }
    rhs.name = node["name"].as<std::string>();
    rhs.override = node["override"].as<double>();
    if (node["corners"])
    {
      rhs.corners = node["corners"].as<std::vector<std::vector<double>>>();
    }
    else if (node["radius"])
    {
      rhs.radius = node["radius"].as<double>();
    }
    else
    {
      return false;
    }
    return true;
  }
};
}  // namespace YAML

class SsmFixedAreasNode :  public SsmBaseNode
{
protected:

  ssm15066::FixedAreasSSMPtr ssm_;
  std::string areas_param_ns_;

  bool loadAreas();

public:

  SsmFixedAreasNode(std::string name);

  bool init() override;

  void spin() override;

};
