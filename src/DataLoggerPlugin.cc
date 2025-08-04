/*
 * Copyright (C) 2025 Open Source Robotics Foundation
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

#include "DataLoggerPlugin.hh"

#include <iostream>
#include <chrono>
#include <iomanip>
#include <cstdlib>
#include <fstream>
#include <functional>

#include <gz/sim/components/Pose.hh>
#include <gz/sim/components/LinearVelocity.hh>
#include <gz/sim/components/LinearAcceleration.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/CanonicalLink.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/Util.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>
#include <gz/math/Pose3.hh>
#include <gz/math/Vector3.hh>
#include <gz/transport/Node.hh>
#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/empty.pb.h>

using namespace gz;
using namespace sim;
using namespace systems;

/// \brief Structure to hold entity information for logging
struct EntityInfo
{
  std::string name;
  std::string type;  // "model" or "link"
  Entity entity{kNullEntity};
  std::unique_ptr<std::ofstream> logFile;
  bool isValid{false};
  
  // For computed acceleration
  math::Vector3d lastVelocity{math::Vector3d::Zero};
  std::chrono::steady_clock::duration lastTime{0};
  bool hasLastVelocity{false};
  
  // Constructor
  EntityInfo() : logFile(std::make_unique<std::ofstream>()) {}
  
  // Move constructor and assignment
  EntityInfo(EntityInfo&& other) = default;
  EntityInfo& operator=(EntityInfo&& other) = default;
  
  // Delete copy constructor and assignment
  EntityInfo(const EntityInfo&) = delete;
  EntityInfo& operator=(const EntityInfo&) = delete;
};

/// \brief Private data class for DataLoggerPlugin
class gz::sim::systems::DataLoggerPluginPrivate
{
  /// \brief Logging frequency in Hz
  public: double logFrequency{100.0};

  /// \brief Output directory for log files
  public: std::string outputDirectory{"/tmp"};

  /// \brief List of entities to log
  public: std::vector<EntityInfo> entities;

  /// \brief Last update time for frequency control
  public: std::chrono::steady_clock::duration lastUpdateTime{0};

  /// \brief Time interval between logs (calculated from frequency)
  public: std::chrono::steady_clock::duration logInterval;

  /// \brief Simulation start time for relative timing
  public: std::chrono::steady_clock::duration simStartTime{0};

  /// \brief Flag to indicate if this is the first update
  public: bool firstUpdate{true};

  /// \brief Transport node for communication
  public: transport::Node node;

  /// \brief Topic name for enabling/disabling logging
  public: std::string enableTopic{"/data_logger/enable"};

  /// \brief Topic name for resetting logs
  public: std::string resetTopic{"/data_logger/reset"};

  /// \brief Flag to control logging state
  public: bool loggingEnabled{false};

  /// \brief Auto-start logging flag
  public: bool autoStart{true};
};

/////////////////////////////////////////////////
DataLoggerPlugin::DataLoggerPlugin()
  : dataPtr(std::make_unique<DataLoggerPluginPrivate>())
{
}

/////////////////////////////////////////////////
DataLoggerPlugin::~DataLoggerPlugin()
{
  // Close all log files
  for (auto &entity : this->dataPtr->entities)
  {
    if (entity.logFile && entity.logFile->is_open())
    {
      entity.logFile->close();
    }
  }
}

/////////////////////////////////////////////////
void DataLoggerPlugin::Configure(const Entity &_entity,
                                const std::shared_ptr<const sdf::Element> &_sdf,
                                EntityComponentManager &_ecm,
                                EventManager &/*_eventMgr*/)
{
  // Parse transport topics
  if (_sdf->HasElement("enable_topic"))
  {
    this->dataPtr->enableTopic = _sdf->Get<std::string>("enable_topic");
  }

  if (_sdf->HasElement("reset_topic"))
  {
    this->dataPtr->resetTopic = _sdf->Get<std::string>("reset_topic");
  }

  // Parse auto-start option
  if (_sdf->HasElement("auto_start"))
  {
    this->dataPtr->autoStart = _sdf->Get<bool>("auto_start");
  }

  // Set initial logging state
  this->dataPtr->loggingEnabled = this->dataPtr->autoStart;

  // Parse log frequency
  if (_sdf->HasElement("log_frequency"))
  {
    this->dataPtr->logFrequency = _sdf->Get<double>("log_frequency");
  }

  // Parse output directory
  if (_sdf->HasElement("output_directory"))
  {
    this->dataPtr->outputDirectory = _sdf->Get<std::string>("output_directory");
  }

  // Calculate log interval from frequency
  this->dataPtr->logInterval = std::chrono::duration_cast<
    std::chrono::steady_clock::duration>(
      std::chrono::duration<double>(1.0 / this->dataPtr->logFrequency));

  // Create output directory if it doesn't exist
  std::string createDirCmd = "mkdir -p " + this->dataPtr->outputDirectory;
  int result = std::system(createDirCmd.c_str());
  if (result != 0)
  {
    gzwarn << "Failed to create output directory: " << this->dataPtr->outputDirectory << std::endl;
  }

  // Subscribe to control topics
  this->dataPtr->node.Subscribe(this->dataPtr->enableTopic, 
    std::function<void(const msgs::Boolean &)>(
    [this](const msgs::Boolean &_msg)
    {
      this->dataPtr->loggingEnabled = _msg.data();
      if (_msg.data())
      {
        // Reset timing when enabling logging
        this->dataPtr->simStartTime = std::chrono::steady_clock::duration{0};
        this->dataPtr->lastUpdateTime = std::chrono::steady_clock::duration{0};
        gzmsg << "DataLogger: Logging ENABLED" << std::endl;
      }
      else
      {
        gzmsg << "DataLogger: Logging DISABLED" << std::endl;
      }
    }));

  this->dataPtr->node.Subscribe(this->dataPtr->resetTopic,
    std::function<void(const msgs::Empty &)>(
    [this](const msgs::Empty &/*_msg*/)
    {
      // Reset timing and clear log files
      this->dataPtr->simStartTime = std::chrono::steady_clock::duration{0};
      this->dataPtr->lastUpdateTime = std::chrono::steady_clock::duration{0};
      
      // Rewrite headers for all log files
      for (auto &entityInfo : this->dataPtr->entities)
      {
        if (entityInfo.logFile && entityInfo.logFile->is_open())
        {
          entityInfo.logFile->close();
          std::string filename = this->dataPtr->outputDirectory + "/" + 
                                entityInfo.name + "_log.csv";
          entityInfo.logFile->open(filename, std::ios::trunc);  // Truncate existing file
          (*entityInfo.logFile) << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z" << std::endl;
          (*entityInfo.logFile) << std::fixed << std::setprecision(6);
        }
        
        // Reset acceleration computation state
        entityInfo.hasLastVelocity = false;
        entityInfo.lastVelocity = math::Vector3d::Zero;
      }
      gzmsg << "DataLogger: Logs RESET and cleared" << std::endl;
    }));

  // Parse entities to log
  if (_sdf->HasElement("entities"))
  {
    // Clone the SDF element to get non-const access
    auto sdfClone = _sdf->Clone();
    auto entitiesElem = sdfClone->GetElement("entities");
    auto entityElem = entitiesElem->GetElement("entity");

    while (entityElem)
    {
      EntityInfo entityInfo;
      
      if (entityElem->HasElement("name"))
      {
        entityInfo.name = entityElem->Get<std::string>("name");
      }
      else
      {
        gzwarn << "Entity element missing 'name' parameter, skipping." << std::endl;
        entityElem = entityElem->GetNextElement("entity");
        continue;
      }

      if (entityElem->HasElement("type"))
      {
        entityInfo.type = entityElem->Get<std::string>("type");
      }
      else
      {
        entityInfo.type = "model";  // Default to model
      }

      this->dataPtr->entities.emplace_back(std::move(entityInfo));
      entityElem = entityElem->GetNextElement("entity");
    }
  }
  else
  {
    gzwarn << "No entities specified for logging. Plugin will not log any data." << std::endl;
    return;
  }

  gzmsg << "DataLoggerPlugin configured with " << this->dataPtr->entities.size() 
        << " entities to log at " << this->dataPtr->logFrequency << " Hz" << std::endl;
  gzmsg << "Output directory: " << this->dataPtr->outputDirectory << std::endl;
  gzmsg << "Enable topic: " << this->dataPtr->enableTopic << std::endl;
  gzmsg << "Reset topic: " << this->dataPtr->resetTopic << std::endl;
  gzmsg << "Auto-start: " << (this->dataPtr->autoStart ? "enabled" : "disabled") << std::endl;
  gzmsg << "Logging initially: " << (this->dataPtr->loggingEnabled ? "enabled" : "disabled") << std::endl;
}

/////////////////////////////////////////////////
void DataLoggerPlugin::PreUpdate(const UpdateInfo &_info,
                                EntityComponentManager &_ecm)
{
  // Initialize entities on first PreUpdate
  if (this->dataPtr->firstUpdate)
  {
    this->dataPtr->firstUpdate = false;
    
    // Find entities and setup log files
    for (auto &entityInfo : this->dataPtr->entities)
    {
      // Find entity by name
      if (entityInfo.type == "model")
      {
        _ecm.Each<components::Model, components::Name>(
          [&](const Entity &_entity,
              const components::Model *,
              const components::Name *_name) -> bool
          {
            if (_name->Data() == entityInfo.name)
            {
              entityInfo.entity = _entity;
              entityInfo.isValid = true;
              
              gzmsg << "Found model entity: " << entityInfo.name 
                    << " with ID: " << entityInfo.entity << std::endl;
              return false;  // Stop searching
            }
            return true;  // Continue searching
          });
      }
      else if (entityInfo.type == "link")
      {
        _ecm.Each<components::Link, components::Name>(
          [&](const Entity &_entity,
              const components::Link *,
              const components::Name *_name) -> bool
          {
            if (_name->Data() == entityInfo.name)
            {
              entityInfo.entity = _entity;
              entityInfo.isValid = true;
              
              gzmsg << "Found link entity: " << entityInfo.name 
                    << " with ID: " << entityInfo.entity << std::endl;
              return false;  // Stop searching
            }
            return true;  // Continue searching
          });
      }

      // Create log file if entity was found
      if (entityInfo.isValid)
      {
        std::string filename = this->dataPtr->outputDirectory + "/" + 
                              entityInfo.name + "_log.csv";
        entityInfo.logFile->open(filename);
        
        if (entityInfo.logFile->is_open())
        {
          // Write CSV header - position, velocity, and acceleration (no rotation)
          (*entityInfo.logFile) << "timestamp,pos_x,pos_y,pos_z,vel_x,vel_y,vel_z,acc_x,acc_y,acc_z" << std::endl;
          (*entityInfo.logFile) << std::fixed << std::setprecision(6);
          gzmsg << "Created log file: " << filename << std::endl;
        }
        else
        {
          gzerr << "Failed to create log file: " << filename << std::endl;
          entityInfo.isValid = false;
        }
      }
      else
      {
        gzwarn << "Could not find entity: " << entityInfo.name << std::endl;
      }
    }
  }

  // Enable components for entities that need them
  for (auto &entityInfo : this->dataPtr->entities)
  {
    if (entityInfo.isValid && entityInfo.entity != kNullEntity)
    {
      std::vector<Entity> entitiesToEnable;
      entitiesToEnable.push_back(entityInfo.entity);
      
      // For model entities, also enable components for canonical link
      if (entityInfo.type == "model")
      {
        auto modelComp = _ecm.Component<components::Model>(entityInfo.entity);
        if (modelComp)
        {
          // Find the canonical link of the model
          _ecm.Each<components::Link, components::CanonicalLink, components::ParentEntity>(
            [&](const Entity &_entity,
                const components::Link *,
                const components::CanonicalLink *,
                const components::ParentEntity *_parent) -> bool
            {
              if (_parent->Data() == entityInfo.entity)
              {
                entitiesToEnable.push_back(_entity);
                // gzmsg << "Found canonical link " << _entity << " for model " << entityInfo.name << std::endl;
                return false; // Stop searching
              }
              return true; // Continue searching
            });
        }
      }
      
      // Enable components for all relevant entities
      for (Entity entity : entitiesToEnable)
      {
        if (!_ecm.EntityHasComponentType(entity, components::WorldPose::typeId))
        {
          _ecm.CreateComponent(entity, components::WorldPose());
        }
        if (!_ecm.EntityHasComponentType(entity, components::WorldLinearVelocity::typeId))
        {
          _ecm.CreateComponent(entity, components::WorldLinearVelocity());
        }
        // Note: We'll compute acceleration by differentiating velocity
      }
    }
  }
}

/////////////////////////////////////////////////
void DataLoggerPlugin::PostUpdate(const UpdateInfo &_info,
                                 const EntityComponentManager &_ecm)
{
  // Skip if paused OR logging disabled
  if (_info.paused || !this->dataPtr->loggingEnabled)
    return;

  // Initialize timing on first PostUpdate
  static bool firstPostUpdate = true;
  if (firstPostUpdate)
  {
    this->dataPtr->simStartTime = _info.simTime;
    this->dataPtr->lastUpdateTime = _info.simTime;
    firstPostUpdate = false;
  }

  // Check if it's time to log
  if (_info.simTime - this->dataPtr->lastUpdateTime < this->dataPtr->logInterval)
  {
    return;
  }

  // Reset start time if this is the first log after enabling
  if (this->dataPtr->simStartTime == std::chrono::steady_clock::duration{0})
  {
    this->dataPtr->simStartTime = _info.simTime;
  }

  this->dataPtr->lastUpdateTime = _info.simTime;

  // Calculate relative simulation time in seconds
  double relativeTime = std::chrono::duration<double>(
    _info.simTime - this->dataPtr->simStartTime).count();

    // Log data for each entity
  for (auto &entityInfo : this->dataPtr->entities)
  {
    if (!entityInfo.isValid || !entityInfo.logFile->is_open())
      continue;

    Entity logEntity = entityInfo.entity;

    // If this is a model, try to get its canonical link for better data
    if (entityInfo.type == "model")
    {
      auto modelComp = _ecm.Component<components::Model>(entityInfo.entity);
      if (modelComp)
      {
        // Try to find the canonical link of the model
        Entity canonicalLink = kNullEntity;
        _ecm.Each<components::Link, components::CanonicalLink, components::ParentEntity>(
          [&](const Entity &_entity,
              const components::Link *,
              const components::CanonicalLink *,
              const components::ParentEntity *_parent) -> bool
          {
            if (_parent->Data() == entityInfo.entity)
            {
              canonicalLink = _entity;
              return false; // Stop searching
            }
            return true; // Continue searching
          });
        
        if (canonicalLink != kNullEntity)
        {
          logEntity = canonicalLink;
        }
        else
        {
          // If no canonical link found directly, try to find nested models
          // This is especially useful for complex models like iris_with_fixed_camera
          _ecm.Each<components::Model, components::ParentEntity>(
            [&](const Entity &_nestedEntity,
                const components::Model *,
                const components::ParentEntity *_parent) -> bool
            {
              if (_parent->Data() == entityInfo.entity)
              {
                // Found a nested model, try to get its canonical link
                _ecm.Each<components::Link, components::CanonicalLink, components::ParentEntity>(
                  [&](const Entity &_linkEntity,
                      const components::Link *,
                      const components::CanonicalLink *,
                      const components::ParentEntity *_linkParent) -> bool
                  {
                    if (_linkParent->Data() == _nestedEntity)
                    {
                      canonicalLink = _linkEntity;
                      return false; // Stop searching
                    }
                    return true; // Continue searching
                  });
                
                if (canonicalLink != kNullEntity)
                {
                  logEntity = canonicalLink;
                  return false; // Stop searching nested models
                }
              }
              return true; // Continue searching
            });
        }
      }
    }

    // Get pose component (try WorldPose first as it's more common)
    auto worldPoseComp = _ecm.Component<components::WorldPose>(logEntity);
    auto poseComp = _ecm.Component<components::Pose>(logEntity);

    // Get velocity component (try WorldLinearVelocity first)
    auto worldVelComp = _ecm.Component<components::WorldLinearVelocity>(logEntity);
    auto velComp = _ecm.Component<components::LinearVelocity>(logEntity);

    // Default values
    math::Pose3d pose = math::Pose3d::Zero;
    math::Vector3d velocity = math::Vector3d::Zero;
    math::Vector3d acceleration = math::Vector3d::Zero;

    // Extract data if components exist (prefer World components)
    if (worldPoseComp)
    {
      pose = worldPoseComp->Data();
    }
    else if (poseComp)
    {
      pose = poseComp->Data();
    }

    if (worldVelComp)
    {
      velocity = worldVelComp->Data();
    }
    else if (velComp)
    {
      velocity = velComp->Data();
    }

    // Compute acceleration by differentiating velocity
    if (entityInfo.hasLastVelocity)
    {
      double dt = std::chrono::duration<double>(_info.simTime - entityInfo.lastTime).count();
      if (dt > 0.0)
      {
        acceleration = (velocity - entityInfo.lastVelocity) / dt;
      }
    }
    
    // Update stored values for next iteration
    entityInfo.lastVelocity = velocity;
    entityInfo.lastTime = _info.simTime;
    entityInfo.hasLastVelocity = true;

    // Write to log file - position, velocity, and acceleration data
    (*entityInfo.logFile) << relativeTime << ","
                      << pose.Pos().X() << "," << pose.Pos().Y() << "," << pose.Pos().Z() << ","
                      << velocity.X() << "," << velocity.Y() << "," << velocity.Z() << ","
                      << acceleration.X() << "," << acceleration.Y() << "," << acceleration.Z() << std::endl;
  }
}

// Register plugin
GZ_ADD_PLUGIN(gz::sim::systems::DataLoggerPlugin,
              gz::sim::System,
              gz::sim::systems::DataLoggerPlugin::ISystemConfigure,
              gz::sim::systems::DataLoggerPlugin::ISystemPreUpdate,
              gz::sim::systems::DataLoggerPlugin::ISystemPostUpdate)

// Add plugin alias
GZ_ADD_PLUGIN_ALIAS(gz::sim::systems::DataLoggerPlugin, "DataLoggerPlugin")
