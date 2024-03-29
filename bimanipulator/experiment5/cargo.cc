#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <nos/print.h>
#include <nos/fprint.h>
#include <nos/terminal.h>

#include <ralgo/linalg/linalg.h>
#include <ralgo/linalg/backpack.h>
#include <ralgo/space/pose2.h>
#include <ralgo/space/screw.h>

#include <igris/math.h>

ralgo::screw2<double> CARGO_TARGET_VELOCITY;
ralgo::pose2<double> CARGO_POSITION;

namespace gazebo
{
	extern physics::WorldPtr WORLD;

	class ModelCargo : public ModelPlugin
	{
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;
		double loctime = 0;

		ralgo::pose2<double> target_position;
		ralgo::pose2<double> current_position;			
		ralgo::pose2<double> error_position;				
		ralgo::screw2<double> error_position_integral;		
		std::fstream fil;

	public:

		double evaltime() 
		{
			auto t = WORLD->SimTime();
			return (double)t.sec + (double)t.nsec / 1000000000.;
		}

		void Reset()
		{
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{		
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&ModelCargo::OnUpdate, this));
			fil.open("cargo.txt", std::ios_base::out);

			Reset();
		}

		// Called by the world update start event
	public:
		double lasttime;
		double starttime;
		double delta;

		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			double time = curtime - starttime;

			auto link0 = model->GetLink("l0");
			auto pos0 = link0->WorldCoGPose().Pos();
			auto rot0 = link0->WorldCoGPose().Rot();

			auto l0pose = 
				ralgo::pose2<double>(0, {pos0.X(), pos0.Z()}) *
				ralgo::pose2<double>(rot0.Pitch(), {0,0});		

			target_position = {0, {sin(evaltime())/2 , 1.2 + cos(evaltime())*0.2}};
			loctime += delta;

			double FT =10;

			if (loctime > FT) loctime = 0;
			if (loctime < FT/4) 
			{
				double koeff = loctime / (FT/4);
				target_position = {0,
					linalg::vec<double,2>{-0.4, 1.0} * koeff +
					linalg::vec<double,2>{0.4, 1.0} * (1-koeff)};
			}
			else
			if (loctime < FT/2) 
			{
				double koeff = (loctime-FT/4) / (FT/4);
				target_position = {0,
					linalg::vec<double,2>{-0.4, 1.3} * koeff +
					linalg::vec<double,2>{-0.4, 1.0} * (1-koeff)};
			}
			else
			if (loctime < FT/4*3) 
			{
				double koeff = (loctime-FT/2) / (FT/4);
				target_position = {0,
					linalg::vec<double,2>{0.4, 1.3} * koeff +
					linalg::vec<double,2>{-0.4, 1.3} * (1-koeff)};
			}
			else
			if (loctime < FT) 
			{
				double koeff = (loctime-FT/4*3) / (FT/4);
				target_position = {0,
					linalg::vec<double,2>{0.4, 1.0} * koeff +
					linalg::vec<double,2>{0.4, 1.3} * (1-koeff)};
			}

			error_position = l0pose.inverse() * target_position;
			ralgo::screw2<double> error_position_screw = {error_position.ang, error_position.lin};

			error_position_integral += error_position_screw * delta;
			if (time < 10) error_position_integral = {0, {0,0}};


			double T = 0.5;
			double ksi = 2;

			auto ki = 1./T/T;
			auto kp = 2.*ksi/T;

			CARGO_TARGET_VELOCITY = 
				error_position_integral * ki +
				error_position_screw * kp;
			CARGO_POSITION = l0pose;

			if (linalg::length(CARGO_TARGET_VELOCITY.lin) > 1) 
			{
				CARGO_TARGET_VELOCITY = CARGO_TARGET_VELOCITY / linalg::length(CARGO_TARGET_VELOCITY.lin);
			}

			lasttime = curtime;

			fil << linalg::length(error_position_screw.lin) << std::endl;
			//if (time > 20) exit(0);
		}
	};
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelCargo)
}