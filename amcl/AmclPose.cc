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

#include <memory>
#include <thread>
#include <string>
#include <algorithm>
#include <string>
#include <vector>
 
#include "AmclPose.hh"

#include <ignition/gazebo/EntityComponentManager.hh>
#include <ignition/gazebo/System.hh>
#include <ignition/common/Profiler.hh>
#include <ignition/plugin/Register.hh>
#include <ignition/transport/Node.hh>
#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <ros/ros.h>
#include <ros/callback_queue.h>

#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::AmclPosePrivate
{
    public: Model model{kNullEntity};

    public: bool updateAmclPose();

    public: std::string fixed_frame_;

    public: std::string robot_frame_;

    public: double rate_;
};

/////////////////////////////////////////////////////////////
AmclPose::AmclPose() : System(), dataPtr(std::make_unique<AmclPosePrivate>()), tfListener_(tfBuffer_)
{
}

/////////////////////////////////////////////////////////////
AmclPose::~AmclPose()
{
   // Disable callback queue
   amcl_queue_.clear();
   amcl_queue_.disable();

   // Shutdown ROS node handle
   amcl_nh_.shutdown();

   // Wait for child thread to join
   if(thread_ptr_->joinable())
   {
      thread_ptr_->join();
   }
}

/////////////////////////////////////////////////////////////
void AmclPose::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
  this->dataPtr->model = Model(_entity); 

  if (!this->dataPtr->model.Valid(_ecm))
  {
     ignerr << "AckermannSteering plugin should be attached to a model entity. "
            << "Failed to initialize." << std::endl;
     return;
  }

  if(_sdf->HasElement("topicRate"))
    {
      auto rate_ = _sdf->Get<double>("topicRate");

    }else{

     rate_ = 10.0;
    }

    if(_sdf->HasElement("fixedFrame"))
    {
      auto fixed_frame_ = _sdf->Get<std::string>("fixedFrame");

    }else{

     fixed_frame_ = "map";
    }

    if(_sdf->HasElement("robotFrame"))    
    {
      auto robot_frame_ = _sdf->Get<std::string>("robotFrame");
        
    }else{
      
     robot_frame_ = "base_link";
    }

    if(!ros::isInitialized())
    {
      int argc = 0; char ** argv = NULL;
      ros::init(argc, argv, "ignition_gazebo_robot", ros::init_options::NoSigintHandler);
    }

    amcl_nh_.setCallbackQueue(& amcl_queue_);
    amcl_pub_ = amcl_nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("AmclPose",1,true);
    thread_ptr_ = std::make_shared<std::thread>([this]() 
            {      
               // Set process rate
               ros::Rate r(this->rate_);
               while (this->amcl_nh_.ok())
               {
                  this->updateAmclPose();
                  this->amcl_pub_.publish(this->msg_);
                  this->amcl_queue_.callAvailable(ros::WallDuration(0.0));
                  r.sleep();
               }
            }
        );
        ROS_INFO_STREAM(ros::this_node::getName() << " plugin amcl pose successfully loaded.");
}

/////////////////////////////////////////////////////////////
void AmclPose::PreUpdate(const UpdateInfo &_info,
    EntityComponentManager &_ecm)
{  
   IGN_PROFILE("AmclPose::PreUpdate");
   this->dataPtr->updateAmclPose();
}

/////////////////////////////////////////////////////////////
bool AmclPose::updateAmclPose()
{
  IGN_PROFILE("AmclPose::updateAmclPose");

   bool isok {false};
   try  
   {
     if(tfBuffer_.canTransform(fixed_frame_, robot_frame_, ros::Time(0))) 
     {
        read_transformation_ = tfBuffer_.lookupTransform(fixed_frame_, robot_frame_, ros::Time(0), ros::Duration(60.0));

        msg_.header.frame_id = fixed_frame_;
        msg_.header.stamp = ros::Time::now();

        msg_.pose.pose.position.x = read_transformation_.transform.translation.x;
        msg_.pose.pose.position.y = read_transformation_.transform.translation.y;
        msg_.pose.pose.position.z = read_transformation_.transform.translation.z;
        msg_.pose.pose.orientation.x = read_transformation_.transform.rotation.x;
        msg_.pose.pose.orientation.y = read_transformation_.transform.rotation.y;
        msg_.pose.pose.orientation.z = read_transformation_.transform.rotation.z;
        msg_.pose.pose.orientation.w = read_transformation_.transform.rotation.w;
     }
     isok = true;     
   } 
  
   catch(const tf2::TimeoutException & e)
   {
     ROS_ERROR_STREAM(ros::this_node::getName() << " " << __func__ <<" timeout exception: " << e.what());    
     return isok;
   }
        
   catch(const tf2::TransformException & e)
   {
     ROS_ERROR_STREAM(ros::this_node::getName() << " " << __func__ <<" transform exception: " << e.what());
     return isok;
   }
   return isok;
}

IGNITION_ADD_PLUGIN(AmclPose,
                    ignition::gazebo::System,
                    AmclPose::ISystemConfigure,
                    AmclPose::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(AmclPose, "ignition::gazebo::systems::AmclPose")