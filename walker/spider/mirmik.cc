#include <mirmik.h>

namespace gazebo
{
	extern physics::WorldPtr WORLD;

	class ModelPush : public ModelPlugin
	{
	private:
		double ForceKoeff = 0.001;
		physics::ModelPtr model;
		event::ConnectionPtr updateConnection;

		int inited = 0;

		double lasttime;
		double starttime;
		double delta;

		BodyController body_controller;

	public:
		double evaltime()
		{
			auto t = WORLD->SimTime();
			return (double)t.sec + (double)t.nsec / 1000000000.;
		}

		void Reset()
		{
			lasttime = evaltime();
			starttime = evaltime();

			inited = 0;
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf)
		{
			(void) _sdf;
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&ModelPush::OnUpdate, this));

			Reset();
		}

	public:

		linalg::vec<double, 2> position_integral {};
		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			double time = curtime - starttime;

			if (!inited)
			{
				body_controller.init(model);

				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;
				lasttime = curtime;

				for (int i = 0; i < 6; ++i) 
				{
					body_controller.legs[i].regulators[1].spd_A = 0.083 * 2;
				}

				return;
			}

			//body_controller.serve(delta);

			nos::println("Control");
			for (int i = 0; i < 1; ++i) {
			/*body_controller.legs[i].regulators[0].speed2_loop_enabled=false;
			body_controller.legs[i].regulators[0].position_loop_enabled=true;
			body_controller.legs[i].regulators[0].position_target=M_PI/8;
			body_controller.legs[i].regulators[0].Control(delta);
			*/
			/*body_controller.legs[i].regulators[1].speed2_loop_enabled=false;
			body_controller.legs[i].regulators[1].position_loop_enabled=true;
			body_controller.legs[i].regulators[1].position_target=M_PI/4;
			body_controller.legs[i].regulators[1].Control(delta);
*/
			body_controller.legs[i].regulators[2].speed2_loop_enabled=false;
			body_controller.legs[i].regulators[2].position_loop_enabled=true;
			body_controller.legs[i].regulators[2].position_target=M_PI/4;
			body_controller.legs[i].regulators[2].Control(delta);
			}
			lasttime = curtime;
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}



gazebo::LegController::LegController(physics::ModelPtr model, int number)
{
	this->relax_pose = relax_pose;

	nos::println("Construct LegController ", number);

	body_link = model->GetLink("body");

	high_link = model->GetLink(nos::format("high_leg_{}", number));
	low_link = model->GetLink(nos::format("low_leg_{}", number));
	fin_link = model->GetLink(nos::format("fin_leg_{}", number));

	shoulder_joint = model->GetJoint(nos::format("joint_shoulder_{}", number));
	high_joint = model->GetJoint(nos::format("joint_high_{}", number));
	low_joint = model->GetJoint(nos::format("joint_low_{}", number));
	fin_joint = model->GetJoint(nos::format("joint_fin_{}", number));

	joints.push_back(shoulder_joint);
	joints.push_back(high_joint);
	joints.push_back(low_joint);

	for (auto j : joints)
		joint_anchors.push_back(
		    rabbit::gazebo_joint_anchor_pose(j));

	for (auto j : joints)
		joint_local_axes.push_back(
		    rabbit::gazebo_joint_local_axis(j));

	for (auto j : joints)
		regulators.emplace_back(j);

	joint_poses.resize(joints.size());
	sensivities.resize(joints.size());

	A_matrix_data.resize(sensivities.size() * 3);
}

void gazebo::LegController::SpeedControl(linalg::vec<double, 3> vec)
{
	mode = LegMode::SpeedMode;
	speed_target = vec;
}

void gazebo::LegController::PositionControl(linalg::vec<double, 3> vec)
{
	mode = LegMode::PositionMode;
	position_target = vec;
}

bool gazebo::LegController::is_landed()
{
	auto fin_reaction = rabbit::gazebo_joint_reaction(fin_joint);
	bool ret = fin_reaction.lin.z > 1e-5;
	return ret;
}

void gazebo::LegController::serve(double delta)
{
	auto fin_local_pose =
	    rabbit::gazebo_link_pose(body_link).inverse() *
	    rabbit::gazebo_link_pose(fin_link);

	for (size_t i = 0; i < joints.size(); ++i)
		joint_coordinates.push_back(joints[i]->Position());

	for (size_t i = 0; i < joints.size(); ++i)
		joint_transes[i] = rabbit::htrans3<double>(
		                       joint_local_axes[i] * joint_coordinates[i]);


	rabbit::htrans3 <double> accumulator;
	joint_poses.push_back(accumulator);
	for (int i = 1; i < joints.size(); ++i)
	{
		accumulator *= joint_anchors[i] * joint_transes[i];
		joint_poses[i] = accumulator;
	}

	for (int i = 1; i < joints.size(); ++i)
	{
		auto body_frame_axis = joint_poses[i]
		                       .rotate_screw(joint_local_axes[i]);

		auto arm = fin_local_pose.lin - joint_poses[i].lin;

		sensivities[i] = body_frame_axis.kinematic_carry(arm);
	}

	ralgo::matrix_view<double> matrix_A(A_matrix_data.data(),
	                                    3, sensivities.size());

	if (mode == LegMode::SpeedMode)
	{
		auto tgt =
		    final_target - fin_local_pose.lin;

		ralgo::svd_backpack(signals, tgt, matrix_A);

		for (int i = 0; i < joints.size(); ++i)
		{
			regulators[i].speed2_target = signals[i];
			regulators[i].Control(delta);
		}
	}
}

gazebo::Regulator::Regulator(physics::JointPtr joint)
	: joint(joint)
{
	update_regs();
}

void gazebo::Regulator::reset()
{
	speed_error = 0;
	position_error = 0;

	speed_target = 0.2;
	position_target = 0;

	speed_integral = 0;
	position_integral = 0;

	speed2_target = 0.1;
}

void gazebo::Regulator::Control(double delta)
{
	double current_position = joint->Position(0);
	double current_speed = joint->GetVelocity(0);

	if (speed2_loop_enabled)
	{
		position_target += speed2_target * delta;
	}

	if (position_loop_enabled)
	{
		position_error = position_target - current_position;
		position_integral += position_error * delta;
		speed_target =
		    pos_kp * position_error +
		    pos_ki * position_integral
		    - control_signal * ForceKoeff;
	}

	nos::println(speed_target);

	speed_error = speed_target - current_speed;
	speed_integral += speed_error * delta;
	control_signal =
	    spd_kp * speed_error +
	    spd_ki * speed_integral;

	joint->SetForce(0, control_signal);
}


gazebo::BodyController::BodyController() :
	trot_controller(this)
{}

void gazebo::BodyController::init(physics::ModelPtr model)
{
	this->model = model;
	legs.clear();

	for (int i = 0; i < 6; ++i)
		legs.emplace_back(model, i);

	for (auto l : legs) l.enable_provide_feedback();

	legs[0].relax_pose = { -1, 1, 0 };
	legs[1].relax_pose = { -1, 0, 0 };
	legs[2].relax_pose = { -1, -1, 0 };
	legs[3].relax_pose = { 1, 1, 0 };
	legs[4].relax_pose = { 1, 0, 0 };
	legs[5].relax_pose = { 1, -1, 0 };
}

void gazebo::BodyController::Control(double delta)
{
	body_pose = rabbit::gazebo_link_pose(model->GetLink("body"));
	body_error = (body_pose.inverse() * body_target).to_screw();

	for (auto l : legs)
	{
		l.SpeedControl({0, 0, 1});
		l.serve(delta);
	}
};

void gazebo::BodyController::TrotControl(double delta)
{
	trot_controller.serve(delta);
}