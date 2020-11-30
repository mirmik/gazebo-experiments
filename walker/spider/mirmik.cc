#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <nos/print.h>
#include <nos/fprint.h>
#include <nos/terminal.h>

#include <linalg/linalg.h>
#include <ralgo/linalg/backpack.h>
#include <rabbit/space/htrans2.h>

#include <igris/math.h>

struct Regulator
{
	bool speed2_loop_enabled = true;
	bool position_loop_enabled = true;

	double speed_error;
	double position_error;

	double speed_target;
	//double position_target = 45. / 180. * 3.14;
	double position_target;

	double speed_integral;
	double position_integral;

	double speed2_target;

	static constexpr double spd_T = 0.05;
	static constexpr double spd_ksi = 0.75;
	static constexpr double spd_A = 9;

	static constexpr double pos_T = 0.2;
	static constexpr double pos_ksi = 2;

	double pos_kp = 2.*pos_ksi/pos_T;
	double pos_ki = 1./pos_T/pos_T;

	double spd_kp = 2.*spd_ksi/spd_T*spd_A;
	double spd_ki = 1./spd_T/spd_T*spd_A;

	//double force_compensation = 0.1;

	double control_signal = 0;

	void reset()
	{
		speed_error = 0;
		position_error = 0;

		speed_target = 0.2;
		position_target = 0;

		speed_integral = 0;
		position_integral = 0;

		speed2_target = 0.1;
	}
};

namespace gazebo
{

	class LegController 
	{
	public:
		physics::LinkPtr high_link;
		physics::LinkPtr low_link;
		physics::LinkPtr fin_link;

		physics::JointPtr shoulder_joint;
		physics::JointPtr high_joint;
		physics::JointPtr low_joint;

		linalg::vec<double,3> target;

		LegController(physics::ModelPtr model, int number) 
		{
			nos::println("Construct LegController ", number);

			high_link = model->GetLink(nos::format("high_link_{}", number));
			low_link = model->GetLink(nos::format("low_link_{}", number));
			fin_link = model->GetLink(nos::format("fin_link_{}", number));

			shoulder_joint = model->GetJoint(nos::format("shoulder_joint_{}", number));
			high_joint = model->GetJoint(nos::format("high_joint_{}", number));
			low_joint = model->GetJoint(nos::format("low_joint_{}", number));
		}	

		void Control(double delta) 
		{
			
		}
	};



	extern physics::WorldPtr WORLD;

	class ModelPush : public ModelPlugin
	{
	private:
		double ForceKoeff = 0.001;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		int inited = 0;
		Regulator joint_base_regulator;
		Regulator joint0_regulator;
		Regulator joint1_regulator;

		double lasttime;
		double starttime;
		double delta;

		std::vector<LegController> legs; 

	public:

		double evaltime() 
		{
			auto t = WORLD->SimTime();
			return (double)t.sec + (double)t.nsec / 1000000000.;
		}

		void Reset()
		{
			joint0_regulator.reset();
			joint1_regulator.reset();

			lasttime = evaltime();

			starttime = evaltime();

			if (model->GetName() == "manip1")
			{
				auto joint0 = model->GetJoint("joint0");
				auto joint1 = model->GetJoint("joint1");
				joint0->SetPosition(0, -3.14 / 4);
				joint1->SetPosition(0, 3.14 / 2);
				joint0_regulator.position_target = -3.14 / 4;
				joint1_regulator.position_target = 3.14 / 2;
			}
			else
			{
				auto joint0 = model->GetJoint("joint0");
				auto joint1 = model->GetJoint("joint1");
				joint0->SetPosition(0, 3.14 / 4);
				joint1->SetPosition(0, -3.14 / 2);
				joint0_regulator.position_target = 3.14 / 4;
				joint1_regulator.position_target = -3.14 / 2;
			}

			inited = 0;
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&ModelPush::OnUpdate, this));

			Reset();
		}

	public:

		linalg::vec<double,2> position_integral {};
		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			double time = curtime - starttime;			

			if (!inited)
			{
				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;

				legs.clear();

				for (int i = 0; i < 6; ++i)
					legs.emplace_back(model, i);

				//auto joint0 = model->GetJoint("joint0");
				//auto joint1 = model->GetJoint("joint1");
				//auto joint2 = model->GetJoint("joint2");
				//joint0->SetProvideFeedback(true);
				//joint1->SetProvideFeedback(true);
				//joint2->SetProvideFeedback(true);

				lasttime = curtime;

				return;
			}



			lasttime = curtime;
		}

		void Control(gazebo::physics::JointPtr joint, Regulator * reg)
		{
			/*double current_position = joint->Position(0);
			double current_speed = joint->GetVelocity(0);

			if (reg -> speed2_loop_enabled)
			{
				reg->position_target += reg->speed2_target * delta;
			}

			if (reg -> position_loop_enabled)
			{
				reg->position_error = reg->position_target - current_position;
				reg->position_integral += reg->position_error * delta;
				reg->speed_target =
				    reg->pos_kp * reg->position_error +
				    reg->pos_ki * reg->position_integral 
				    - reg->control_signal * ForceKoeff;
			}

			PRINT(reg->speed_target);
			reg->speed_error = reg->speed_target - current_speed;
			reg->speed_integral += reg->speed_error * delta;
			reg->control_signal =
			    reg->spd_kp * reg->speed_error +
			    reg->spd_ki * reg->speed_integral;

			joint->SetForce(0, reg->control_signal);*/
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}