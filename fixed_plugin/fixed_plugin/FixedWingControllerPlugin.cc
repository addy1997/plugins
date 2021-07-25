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

#include <array>
#include <chrono>
#include <functional>
#include <mutex>
#include <string>

#include <ignition/common/Profiler.hh>

#include "ignition/gazebo/components/JointForce.hh"
#include "ignition/gazebo/components/JointForceCmd.hh"
#include "ignition/gazebo/components/JointPosition.hh"
#include "ignition/gazebo/components/JointVelocity.hh"
#include "ignition/gazebo/components/JointVelocityCmd.hh"

#include "ignition/gazebo/Model.hh"
#include "ignition/gazebo/Util.hh"

#include <ignition/math/PID.hh>

#include <ignition/msgs.hh>
#include <ignition/msgs/cessna.pb.h>
#include <ignition/msgs/double.pb.h>

#include <ignition/plugin/Register.hh>

#include <ignition/transport/Node.hh>

#include <sdf/sdf.hh>

#include "FixedWingControllerPlugin.hh"

using namespace ignition;
using namespace gazebo;
using namespace systems;

class ignition::gazebo::systems::FixedWingControllerPluginPrivate
{  
   /// \brief Update PID Joint controllers.
   /// \param[in,out] _ecm Ignition Entity Component Manager
   /// \param[in] _dt time step size since last update.
   public: void UpdatePIDs(const EntityComponentManager &_ecm, double _dt);

   /// \brief Publish Cessna state.
   public: void PublishState(const EntityComponentManager &_ecm);

   /// \brief Node for ignition communications.
   public: ignition::transport::Node node;

   /// \brief Publisher node.
   public: ignition::transport::Node::Publisher statePub;

   /// \brief Interface to the model;
   public: Model model{kNullEntity};

   /// \brief Controller update mutex.
   public: std::mutex mutex;

   /// \brief keep track of controller update sim-time.
   public: std::chrono::steady_clock::duration lastControllerUpdateTime{0};

   /// \brief Position PID for the control surfaces.
   public: std::array<ignition::math::PID, 6> controlSurfacesPID;

   /// \brief Velocity PID for the propeller.
   public: ignition::math::PID propellerPID;

   /// \brief Next command to be applied to the propeller and control surfaces.
   public: std::array<float, 7> cmds;

   /// \brief Max propeller RPM.
   public: int32_t propellerMaxRpm = 2500;

   /// \brief Control surfaces joints.
   public: std::array<Entity,7> joints;

   /// \brief Latest update info
   public: UpdateInfo updateInfo;

   /// \brief Joint Entity
   public: Entity jointEntity;

   /// \brief Joint name
   public: std::string jointName;

   /// \brief Commanded joint position
   public: double jointPosCmd;

   /// \brief Command joint velocity
   public: double jointVelCmd;

   /// \brief Joint indices
   public: static const unsigned int kLeftAileron  = 0;
   public: static const unsigned int kLeftFlap     = 1;
   public: static const unsigned int kRightAileron = 2;
   public: static const unsigned int kRightFlap    = 3;
   public: static const unsigned int kElevators    = 4;
   public: static const unsigned int kRudder       = 5;
   public: static const unsigned int kPropeller    = 6;
};

///////////////////////////////////////////////////////////////////////////
FixedWingControllerPlugin::FixedWingControllerPlugin()
    : dataPtr(std::make_unique<FixedWingControllerPluginPrivate>())
{    
   this->dataPtr->cmds.fill(0.0f);

   // PID default parameters.
   this->dataPtr->propellerPID.Init(50.0, 0.1, 1, 0.0, 0.0, 20000.0, -20000.0);
   this->dataPtr->propellerPID.SetCmd(0.0);

   for (auto &pid : this->dataPtr->controlSurfacesPID)
   {    
      pid.Init(50.0, 0.1, 1, 0.0, 0.0, 20.0, -20.0);
      pid.SetCmd(0.0);
   }
}

///////////////////////////////////////////////////////////////////////////
FixedWingControllerPlugin::~FixedWingControllerPlugin()
{
}

///////////////////////////////////////////////////////////////////////////
void FixedWingControllerPlugin::Configure(const Entity &_entity,
    const std::shared_ptr<const sdf::Element> &_sdf,
    EntityComponentManager &_ecm,
    EventManager &_eventMgr)
{
   this->dataPtr->model = ignition::gazebo::Model(_entity);

   if (!this->dataPtr->model.Valid(_ecm))
   {
      ignerr << "Cessna plugin should be attached to a model "
             << "entity. Failed to initialize." << std::endl;
      return;
   }

   // Read the required parameter for the propeller max RPMs.
   if (!_sdf->HasElement("propeller_max_rpm"))
   {  
      ignerr << "Unable to find the <propeller_max_rpm> parameter." << std::endl;
      return;
   }
   this->dataPtr->propellerMaxRpm = _sdf->Get<int32_t>("propeller_max_rpm");
   if (this->dataPtr->propellerMaxRpm == 0)
   {
      ignerr << "Maximum propeller RPMs cannot be 0" << std::endl;
      return;
   }

   // Get joint name from SDF
   this->dataPtr->jointName = _sdf->Get<std::string>("joint_name");

   if (this->dataPtr->jointName == "")
   {
      ignerr << "FixedWingController found an empty jointName parameter. "
             << "Failed to initialize.";
      return;
   }

   // If the joint hasn't been identified yet, look for it
   if (this->dataPtr->jointEntity == kNullEntity)
   {
      this->dataPtr->jointEntity =
        this->dataPtr->model.JointByName(_ecm, this->dataPtr->jointName);
   }

   if (this->dataPtr->jointEntity == kNullEntity)
      return;
   
   // Overload the PID parameter if they are available
   if (_sdf->HasElement("propeller_p_gain"))
     this->dataPtr->propellerPID.SetPGain(_sdf->Get<double>("propeller_p_gain"));

   if (_sdf->HasElement("propeller_i_gain"))
     this->dataPtr->propellerPID.SetIGain(_sdf->Get<double>("propeller_i_gain"));

   if (_sdf->HasElement("propeller_d_gain"))
     this->dataPtr->propellerPID.SetDGain(_sdf->Get<double>("propeller_d_gain"));
    
   if (_sdf->HasElement("surfaces_p_gain"))
   {  
      for (auto &pid : this->dataPtr->controlSurfacesPID)
      pid.SetPGain(_sdf->Get<double>("surfaces_p_gain"));
   }

   if (_sdf->HasElement("surfaces_i_gain"))
   {  
      for (auto &pid : this->dataPtr->controlSurfacesPID)
      pid.SetIGain(_sdf->Get<double>("surfaces_i_gain"));
   }

   if (_sdf->HasElement("surfaces_d_gain"))
   {
      for (auto &pid : this->dataPtr->controlSurfacesPID)
      pid.SetDGain(_sdf->Get<double>("surfaces_d_gain"));
   }

   auto OnControlCb = std::function<void(const msgs::Cessna &)>(
      [this](const auto &_msg)
      {
         std::lock_guard<std::mutex> lock(this->dataPtr->mutex);

         if (_msg.cmd_propeller_speed() && std::abs(_msg.cmd_propeller_speed()) <= 1)
         {  
            this->dataPtr->cmds[this->dataPtr->kPropeller] = _msg.cmd_propeller_speed();
         }
         if (_msg.cmd_left_aileron() == true)
            this->dataPtr->cmds[this->dataPtr->kLeftAileron] = _msg.cmd_left_aileron();
          
         if (_msg.cmd_left_flap() == true)
            this->dataPtr->cmds[this->dataPtr->kLeftFlap] = _msg.cmd_left_flap();
        
         if (_msg.cmd_right_aileron() == true)
            this->dataPtr->cmds[this->dataPtr->kRightAileron] = _msg.cmd_right_aileron();
        
         if (_msg.cmd_right_flap() == true)
            this->dataPtr->cmds[this->dataPtr->kRightFlap] = _msg.cmd_right_flap();
        
         if (_msg.cmd_elevators() == true)
            this->dataPtr->cmds[this->dataPtr->kElevators] = _msg.cmd_elevators();

         if (_msg.cmd_rudder() == true)
            this->dataPtr->cmds[this->dataPtr->kRudder] = _msg.cmd_rudder();  
      });

   std::string prefix = this->dataPtr->model.Name(_ecm) + "/";
   this->dataPtr->statePub = this->dataPtr->node.Advertise<ignition::msgs::Cessna>
                              (prefix + "state");
   this->dataPtr->node.Subscribe("control"+prefix,OnControlCb);

   ignlog <<"Cessna ready to fly. The force will be with you"<< std::endl;
}

///////////////////////////////////////////////////////////////////////////
void FixedWingControllerPlugin::PreUpdate(const UpdateInfo &_info, 
   EntityComponentManager &_ecm)
{      
   IGN_PROFILE("FixedWingControllerPlugin::PreUpdate");
   std::lock_guard<std::mutex> lock(this->dataPtr->mutex); 

   if (_info.simTime > this->dataPtr->lastControllerUpdateTime)
   {
      // Update the control surfaces and publish the new state.
      IGN_PROFILE_BEGIN("Update");
      this->dataPtr->UpdatePIDs(
         _ecm, std::chrono::duration_cast<std::chrono::duration<double> >(
            _info.simTime - this->dataPtr->lastControllerUpdateTime).count());
      IGN_PROFILE_END();
      IGN_PROFILE_BEGIN("Publish");
      this->dataPtr->PublishState(_ecm);
      IGN_PROFILE_END();

      this->dataPtr->lastControllerUpdateTime = _info.simTime;
   }  
}

///////////////////////////////////////////////////////////////////////////
void FixedWingControllerPluginPrivate::UpdatePIDs(
   const EntityComponentManager &_ecm, double _dt)
{ 
   // Velocity PID for propeller
   ignition::gazebo::UpdateInfo &_info(this->updateInfo);
}

///////////////////////////////////////////////////////////////////////////
void FixedWingControllerPluginPrivate::PublishState( 
   const EntityComponentManager &_ecm)
{
   // Current position of joints
   auto propellerRpms = _ecm.Component<components::JointVelocity>
   (this->joints[this->kPropeller])->Data().at(0);
   double propellerSpeed = propellerRpms / this->propellerMaxRpm;
   auto leftAileron = _ecm.Component<components::JointPosition>
   (this->joints[this->kLeftAileron])->Data().at(0);
   auto leftFlap = _ecm.Component<components::JointPosition>
   (this->joints[this->kLeftFlap])->Data().at(0);
   auto rightAileron = _ecm.Component<components::JointPosition>
   (this->joints[this->kRightAileron])->Data().at(0);
   auto rightFlap = _ecm.Component<components::JointPosition>
   (this->joints[this->kRightFlap])->Data().at(0);
   auto elevators = _ecm.Component<components::JointPosition>
   (this->joints[this->kElevators])->Data().at(0);
   auto rudder =  _ecm.Component<components::JointPosition>
   (this->joints[this->kRudder])->Data().at(0);

   ignition::msgs::Cessna msg;
   // Set the observed state.
   msg.set_propeller_speed(propellerSpeed);
   msg.set_left_aileron(leftAileron);
   msg.set_left_flap(leftFlap);
   msg.set_right_aileron(rightAileron);
   msg.set_right_flap(rightFlap);
   msg.set_elevators(elevators);
   msg.set_rudder(rudder);

   // Set the target state.
   msg.set_cmd_propeller_speed(this->cmds[this->kPropeller]);
   msg.set_cmd_left_aileron(this->cmds[this->kLeftAileron]);
   msg.set_cmd_left_flap(this->cmds[this->kLeftFlap]);
   msg.set_cmd_right_aileron(this->cmds[this->kRightAileron]);
   msg.set_cmd_right_flap(this->cmds[this->kRightFlap]);
   msg.set_cmd_elevators(this->cmds[this->kElevators]);
   msg.set_cmd_rudder(this->cmds[this->kRudder]);

   this->statePub.Publish(msg);
}

// Register Plugin
IGNITION_ADD_PLUGIN(FixedWingControllerPlugin,
                    ignition::gazebo::System,
                    FixedWingControllerPlugin::ISystemConfigure,
                    FixedWingControllerPlugin::ISystemPreUpdate)
                
IGNITION_ADD_PLUGIN_ALIAS(FixedWingControllerPlugin, 
   "ignition::gazebo::systems::FixedWingControllerPlugin")