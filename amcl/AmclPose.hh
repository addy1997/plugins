/*
 * Copyright (C) 2021 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 */

#ifndef IGNITION_GAZEBO_AMCL_PLUGIN_HH
#define IGNITION_GAZEBO_AMCL_PLUGIN_HH

#include <memory>

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/Util.hh>
#include <ignition/gazebo/System.hh>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

#include <ros/ros.h>
#include <ros/callback_queue.h>

namespace ignition
{
namespace gazebo
{
// Inline bracket to help doxygen filtering.
inline namespace IGNITION_GAZEBO_VERSION_NAMESPACE {
namespace systems
{
  // Forward declaration
  class AmclPosePrivate;

  
  class AmclPose
      : public System,
        public ISystemConfigure,
        public ISystemPreUpdate
  {
    /// \brief Constructor
    public: AmclPose();

    /// \brief Destructor
    public: ~AmclPose() override;

    // Documentation inherited
    public: void Configure(const Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           EntityComponentManager &_ecm,
                           EventManager &_eventMgr) override;

    public: void PreUpdate(const UpdateInfo &_info,
                           EntityComponentManager &_ecm);

    public: tf2_ros::Buffer tfBuffer_;

    public: tf2_ros::TransformListener tfListener_;

    public: double rate_;

    public: ros::NodeHandle amcl_nh_;

    public: ros::CallbackQueue amcl_queue_;

    public: ros::Publisher amcl_pub_;

    public: std::string fixed_frame_;

    public: std::string robot_frame_;

    public: std::shared_ptr<std::thread> thread_ptr_;

    public: geometry_msgs::PoseWithCovarianceStamped msg_;

    public: geometry_msgs::TransformStamped read_transformation_;

    public: bool updateAmclPose();
    
    public: Model model{kNullEntity};

    /// \brief Private data pointer
    private: std::unique_ptr<AmclPosePrivate> dataPtr;
  };
  }
}
}
}

#endif