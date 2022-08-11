#include <gazebo/gazebo.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/transport/transport.hh>
#include <igris/util/string.h>
#include <nos/print.h>
#include <nos/terminal.h>

namespace gazebo
{
    physics::WorldPtr WORLD;

    class RemoteControlWorldPlugin : public WorldPlugin
    {
        event::ConnectionPtr updateConnection;
        gazebo::transport::PublisherPtr hello_pub;
        gazebo::transport::NodePtr node;

    public:
        RemoteControlWorldPlugin() : WorldPlugin() {}

    public:
        void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf) override
        {
            nos::fprintln("Load world: {}", _world->Name());
            WORLD = _world;

            node = transport::NodePtr(new gazebo::transport::Node());
            node->Init();

            hello_pub = node->Advertise<gazebo::msgs::GzString>("~/hello");

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RemoteControlWorldPlugin::OnUpdate, this));
        }

        void OnUpdate()
        {
            gazebo::msgs::GzString msg;
            msg.set_data("hello from " + WORLD->Name());
            hello_pub->Publish(msg);
        }

        ~RemoteControlWorldPlugin() {}
    };
    GZ_REGISTER_WORLD_PLUGIN(RemoteControlWorldPlugin)
}