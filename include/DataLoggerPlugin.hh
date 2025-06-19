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
#ifndef DATALOGGERPLUGIN_HH_
#define DATALOGGERPLUGIN_HH_

#include <memory>
#include <string>
#include <fstream>

#include <gz/sim/System.hh>
#include <sdf/sdf.hh>

namespace gz
{
namespace sim
{
namespace systems
{
// Forward declare private data class
class DataLoggerPluginPrivate;

/// \brief Data Logger Plugin
/// 
/// This plugin logs position, velocity, and linear acceleration data
/// for specified entities to CSV files. It can target specific entities
/// by name or entity ID.
///
/// Parameters:
///   <log_frequency>   Logging frequency in Hz (default: 100)
///   <output_directory> Directory to store log files (default: /tmp)
///   <entities>        List of entity names to log data for
///     <entity>
///       <name>          Entity name to search for
///       <type>          Entity type (model or link, default: model)
///     </entity>
///   </entities>
///
/// Example usage:
/// <plugin filename="libDataLoggerPlugin.so" name="gz::sim::systems::DataLoggerPlugin">
///   <log_frequency>50</log_frequency>
///   <output_directory>/tmp</output_directory>
///   <entities>
///     <entity>
///       <name>iris_with_fixed_camera</name>
///       <type>model</type>
///     </entity>
///     <entity>
///       <name>moving_platform</name>
///       <type>model</type>
///     </entity>
///   </entities>
/// </plugin>
class GZ_SIM_VISIBLE DataLoggerPlugin:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate,
  public gz::sim::ISystemPostUpdate
{
  /// \brief Constructor.
  public: DataLoggerPlugin();

  /// \brief Destructor.
  public: ~DataLoggerPlugin();

  /// \brief Configure the system.
  public: void Configure(const gz::sim::Entity &_entity,
                        const std::shared_ptr<const sdf::Element> &_sdf,
                        gz::sim::EntityComponentManager &_ecm,
                        gz::sim::EventManager &_eventMgr) override;

  /// \brief Do the part of one update loop that involves making
  ///        changes to simulation.
  public: void PreUpdate(const gz::sim::UpdateInfo &_info,
                          gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Do the part of one update loop that involves
  ///        reading results from simulation.
  public: void PostUpdate(const gz::sim::UpdateInfo &_info,
                          const gz::sim::EntityComponentManager &_ecm) override;

  /// \brief Private data pointer.
  private: std::unique_ptr<DataLoggerPluginPrivate> dataPtr;
};

}  // namespace systems
}  // namespace sim
}  // namespace gz

#endif  // DATALOGGERPLUGIN_HH_
