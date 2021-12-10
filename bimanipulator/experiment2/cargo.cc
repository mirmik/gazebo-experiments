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
#include <rabbit/space/htrans2.h>

#include <igris/math.h>

rabbit::screw2<double> CARGO_TARGET_VELOCITY;
rabbit::htrans2<double> CARGO_POSITION;

namespace gazebo
{
	extern physics::WorldPtr WORLD;

	class ModelCargo : public ModelPlugin
	{
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		rabbit::htrans2<double> target_position;
		rabbit::htrans2<double> current_position;			
		rabbit::htrans2<double> error_position;				
		rabbit::screw2<double> error_position_integral;		

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
			nos::println("Load cargo");
		
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&ModelCargo::OnUpdate, this));

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
				rabbit::htrans2<double>(0, {pos0.X(), pos0.Z()}) *
				rabbit::htrans2<double>(rot0.Pitch(), {0,0});		

			target_position = {0, {sin(evaltime())/2 , 1.2 + cos(evaltime())*0.2}};
			//target_position = {0, {0,1}};
			error_position = l0pose.inverse() * target_position;
			rabbit::screw2<double> error_position_screw = {error_position.orient, error_position.center};

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

			nos::println(error_position);
			lasttime = curtime;
		}
	};
	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelCargo)
}