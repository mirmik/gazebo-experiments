#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <mosquitto_client/mqtt.h>
#include <mosquittopp.h>
#include <nos/print.h>
#include <nos/terminal.h>
#include <nos/util/string.h>

mqtt_client *MQTT;

namespace gazebo
{
    physics::WorldPtr WORLD;
    mqtt_client *client;

    class WorldCtr : public WorldPlugin
    {
    private:
        void command_handle(std::string msg)
        {
            msg = nos::trim(msg);
            nos::fprintln("command: {}", msg);
            if (msg == "restart")
            {
                nos::println("world restarted");
                WORLD->Reset();
            }
            else if (msg == "hello")
                nos::println("Hello, world!");
            else
            {
                nos::fprintln("Unknown command: {}", msg);
            }
        }

    public:
        WorldCtr() : WorldPlugin()
        {
            printf("WorldCtr plugin loaded.\n");
            client = new mqtt_client("worldctr", "localhost", 1883);
            MQTT = client;
            MQTT->loop_start();

            MQTT->subscribe("/worldctr/command",
                            [this](const std::string &msg)
                            { this->command_handle(msg); });
        }

        virtual void Reset() {}

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
        {
            WORLD = _world;
        }
    };
    GZ_REGISTER_WORLD_PLUGIN(WorldCtr)
}