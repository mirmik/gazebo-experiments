#include <gazebo/gazebo.hh>
#include <nos/print.h>
#include <nos/terminal.h>

#include <crow/gates/udpgate.h>
#include <crow/nodes/publisher_node.h>
#include <crow/address.h>
#include <crow/tower.h>

auto crowker = crow::address(".12.127.0.0.1:10009");

namespace gazebo
{
	physics::WorldPtr WORLD;

	class WorldPluginTutorial : public WorldPlugin
	{
		crow::publisher_node publisher;

	public:
		WorldPluginTutorial() : WorldPlugin()
		{
			crow::create_udpgate(12);
			crow::start_spin();
			publisher.init(crowker, "sim");
		}

		void Reset() override
		{
			nos::reset_terminal();
			nos::println("world_init");
			publisher.publish("reset");
			nos::println("world_init .. ok");
		}

	public:
		void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
		{
			WORLD = _world;
			Reset();
		}
	};
	GZ_REGISTER_WORLD_PLUGIN(WorldPluginTutorial)
}