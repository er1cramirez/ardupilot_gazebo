#include <gz/sim/System.hh>
#include <gz/plugin/Register.hh>
#include <gz/common/Console.hh>

namespace gz::sim::v8::systems
{
    class TestPlugin : public System,
                       public ISystemConfigure
    {
        public: void Configure(const Entity &,
                             const std::shared_ptr<const sdf::Element> &,
                             EntityComponentManager &,
                             EventManager &) override
        {
            gzmsg << "TestPlugin loaded and configured!" << std::endl;
        }
    };
}

GZ_ADD_PLUGIN(gz::sim::v8::systems::TestPlugin,
              gz::sim::System,
              gz::sim::v8::systems::TestPlugin::ISystemConfigure)
