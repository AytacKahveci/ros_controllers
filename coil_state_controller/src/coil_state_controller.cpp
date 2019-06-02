///////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2012, hiDOF INC.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//   * Redistributions of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//   * Redistributions in binary form must reproduce the above copyright
//     notice, this list of conditions and the following disclaimer in the
//     documentation and/or other materials provided with the distribution.
//   * Neither the name of hiDOF Inc nor the names of its
//     contributors may be used to endorse or promote products derived from
//     this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//////////////////////////////////////////////////////////////////////////////

/*
 * Author: Aytac Kahveci
 */

#include <algorithm>
#include <cstddef>

#include "coil_state_controller/coil_state_controller.h"

namespace coil_state_controller
{

  bool CoilStateController::init(hardware_interface::CurrentSensorInterface* hw,
                                  ros::NodeHandle&                         root_nh,
                                  ros::NodeHandle&                         controller_nh)
  {
    // get all joint names from the hardware interface
    const std::vector<std::string>& coil_names = hw->getNames();
    num_hw_coils_ = coil_names.size();
    for (unsigned i=0; i<num_hw_coils_; i++)
      ROS_DEBUG("Got joint %s", coil_names[i].c_str());

    // get publishing period
    if (!controller_nh.getParam("publish_rate", publish_rate_)){
      ROS_ERROR("Parameter 'publish_rate' not set");
      return false;
    }

    // realtime publisher
    realtime_pub_.reset(new realtime_tools::RealtimePublisher<coil_states_msgs::CoilStates>(root_nh, "coil_states", 4));

    // get joints and allocate message
    for (unsigned i=0; i<num_hw_coils_; i++){
      coil_state_.push_back(hw->getHandle(coil_names[i]));
      realtime_pub_->msg_.name.push_back(coil_names[i]);
      realtime_pub_->msg_.current.push_back(0.0);
      realtime_pub_->msg_.direction.push_back(1.0);
    }
    addExtraJoints(controller_nh, realtime_pub_->msg_);

    return true;
  }

  void CoilStateController::starting(const ros::Time& time)
  {
    // initialize time
    last_publish_time_ = time;
  }

  void CoilStateController::update(const ros::Time& time, const ros::Duration& /*period*/)
  {
    // limit rate of publishing
    if (publish_rate_ > 0.0 && last_publish_time_ + ros::Duration(1.0/publish_rate_) < time){

      // try to publish
      if (realtime_pub_->trylock()){
        // we're actually publishing, so increment time
        last_publish_time_ = last_publish_time_ + ros::Duration(1.0/publish_rate_);

        // populate joint state message:
        // - fill only joints that are present in the JointStateInterface, i.e. indices [0, num_hw_joints_)
        // - leave unchanged extra joints, which have static values, i.e. indices from num_hw_joints_ onwards
        realtime_pub_->msg_.header.stamp = time;
        for (unsigned i=0; i<num_hw_coils_; i++){
          realtime_pub_->msg_.current[i] = coil_state_[i].getCurrent();
          realtime_pub_->msg_.direction[i] = coil_state_[i].getDirection();
        }
        realtime_pub_->unlockAndPublish();
      }
    }
  }

  void CoilStateController::stopping(const ros::Time& /*time*/)
  {}

  void CoilStateController::addExtraJoints(const ros::NodeHandle& nh, coil_states_msgs::CoilStates& msg)
  {

    // Preconditions
    XmlRpc::XmlRpcValue list;
    if (!nh.getParam("extra_coils", list))
    {
      ROS_DEBUG("No extra coils specification found.");
      return;
    }

    if (list.getType() != XmlRpc::XmlRpcValue::TypeArray)
    {
      ROS_ERROR("Extra coils specification is not an array. Ignoring.");
      return;
    }

    for(std::size_t i = 0; i < list.size(); ++i)
    {
      if (list[i].getType() != XmlRpc::XmlRpcValue::TypeStruct)
      {
        ROS_ERROR_STREAM("Extra coils specification is not a struct, but rather '" << list[i].getType() <<
                         "'. Ignoring.");
        continue;
      }

      if (!list[i].hasMember("name"))
      {
        ROS_ERROR_STREAM("Extra coils does not specify name. Ignoring.");
        continue;
      }

      const std::string name = list[i]["name"];
      if (std::find(msg.name.begin(), msg.name.end(), name) != msg.name.end())
      {
        ROS_WARN_STREAM("Joint state interface already contains specified extra joint '" << name << "'.");
        continue;
      }

      const bool has_current = list[i].hasMember("current");
      const bool has_direction = list[i].hasMember("direction");

      const XmlRpc::XmlRpcValue::Type typeDouble = XmlRpc::XmlRpcValue::TypeDouble;
      if (has_current && list[i]["current"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra coil '" << name << "' does not specify a valid default position. Ignoring.");
        continue;
      }
      if (has_direction && list[i]["direction"].getType() != typeDouble)
      {
        ROS_ERROR_STREAM("Extra coil '" << name << "' does not specify a valid default velocity. Ignoring.");
        continue;
      }

      // State of extra joint
      const double current = has_current ? static_cast<double>(list[i]["current"]) : 0.0;
      const double direction = has_direction ? static_cast<double>(list[i]["direction"]) : 0.0;

      // Add extra joints to message
      msg.name.push_back(name);
      msg.current.push_back(current);
      msg.direction.push_back(direction);
    }
  }

}

PLUGINLIB_EXPORT_CLASS( coil_state_controller::CoilStateController, controller_interface::ControllerBase)
