/*
*
* Author: Adwait Naik | Github:addy1997
* Date: April 12, 2021
* Desc: A simple dummy plugin for gripper module
*/

#include "Gripper.hh"

#include <algorithm>
#include <string>
#include <vector>

#include <ignition/plugin/Register.hh>
#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/Link.hh"
#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include "ros/ros.h"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::GripperPluginPrivate
{
   // access point
   public: ros::NodeHandle node;

   // Publish topic
   public: ros::Publisher pub;

   // Subscribe to the topic
   public: ros::Subscriber sub;
    
   public: std::mutex mutex;
};

//////////////////////////////////////////////////////////////////////////////
GripperPlugin::GripperPlugin():
  System(), dataPtr(std::make_unique<GripperPluginPrivate>())
{
  ignmsg << "[Gripper] loaded gripper plugin." <<std::endl;
}

///////////////////////////////////////////////////////////////////////////////
GripperPlugin::~GripperPlugin()
{
  ignmsg << "[Gripper] unloaded gripper plugin." <<std::endl;
}

//////////////////////////////////////////////////////////////////////////////////
void GripperPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
   this->model = Model(_entity);

   if (!this->model.Valid(_ecm))
   {
      ignerr << "Gripper should be attached to a model entity."
             << "Failed to initialize."<< std::endl;
      return;
   }

    //Check for joint count
    if (model.JointCount(_ecm) == 0)
    {
      ignerr << "[Gripper] Invalid model."<< std::endl;
    }
}

//////////////////////////////////////////////////////////////////////////////////////////
void GripperPlugin::PreUpdate(const UpdateInfo &_info,
                         EntityComponentManager &_ecm)
{
   IGN_PROFILE("GripperPlugin::PostUpdate");

   if (_info.dt < std::chrono::steady_clock::duration::zero())
   {
      ignwarn << "Detected jump back in time ["
              <<     std::chrono::duration_cast<std::chrono::seconds>(_info.dt).count()
              << "s]. System may not work properly."<< std::endl;
   }
}

IGNITION_ADD_PLUGIN(GripperPlugin,
                    ignition::gazebo::System,
                    GripperPlugin::ISystemConfigure,
                    GripperPlugin::ISystemPreUpdate)

IGNITION_ADD_PLUGIN_ALIAS(GripperPlugin, "ignition::gazebo::systems::GripperPlugin")
