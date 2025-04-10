/*
Copyright (c) 2025, Marco Faroni
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

#include "ssm_safety_ros/ssm_ros_signals_node_library.h"

SsmSignalsNode::SsmSignalsNode(std::string name, std::string configuration): SsmBaseNode(name)
{
  params_ns_ = "/"+name+"/";
  signals_ns_ = params_ns_+configuration;
}

bool SsmSignalsNode::init()
{

  // get params
  std::string what;

  std::vector<Signal> signals;
  if (!cnr::param::get(signals_ns_, signals, what))
  {
    RCLCPP_ERROR_STREAM(this->get_logger(), "could not load parameter /signals." << what);
    return false;
  }

  signals_handlers_.clear();

  for (const auto& signal: signals)
  {
    SignalHandlerPtr signal_handler;
    if (!signal.interface.compare("msg"))
    {
      signal_handler = std::make_shared<TopicHandler>(signal, shared_from_this());
    }
    else if (!signal.interface.compare("srv"))
    {
      signal_handler = std::make_shared<ServiceHandler>(signal, shared_from_this());
    }
    else
    {
      RCLCPP_ERROR_STREAM(this->get_logger(), "Signal interface undefined. Options are srv and msg." << what);
      return false;
    }
    signals_handlers_.push_back(signal_handler);
  }

  for (auto& signal_handler: signals_handlers_)
  {
    signal_handler->init();
  }

  RCLCPP_INFO(this->get_logger(), "ssm_signals_node initialized");

  return true;
}

void SsmSignalsNode::spin()
{
    RCLCPP_DEBUG(this->get_logger(), "hello world");

    rclcpp::WallRate lp(1.0/sampling_time_);
    while (rclcpp::ok())
    {
      double ovr = 1.0;
      for (const auto& signal: signals_handlers_)
      {
        if (signal->is_active())
        {
          //ovr = std::min(ovr,signal->get_target_override()); // take the minimum among all active signals
        }
      }
      ovr = std::max(0.0, ovr);
      publish_ovr(ovr);
      lp.sleep();
    }

}




