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

#include <rabbit/space/htrans3.h>
#include <rabbit/space/gazebo.h>
#include <igris/math.h>

namespace gazebo
{
	class Regulator
	{
	public:
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

		double pos_kp = 2.*pos_ksi / pos_T;
		double pos_ki = 1. / pos_T / pos_T;

		double spd_kp = 2.*spd_ksi / spd_T * spd_A;
		double spd_ki = 1. / spd_T / spd_T * spd_A;

		physics::JointPtr joint;
		//double force_compensation = 0.1;

		double control_signal = 0;
		double ForceKoeff = 0.1;

		Regulator(physics::JointPtr joint)
			: joint(joint)
		{}

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

		void Control(double delta)
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

			speed_error = speed_target - current_speed;
			speed_integral += speed_error * delta;
			control_signal =
			    spd_kp * speed_error +
			    spd_ki * speed_integral;

			joint->SetForce(0, control_signal);
		}
	};


	class LegController
	{
		enum class LegMode
		{
			SpeedMode,
			PositionMode
		};

	public:
		LegMode mode;

		physics::LinkPtr body_link;

		physics::LinkPtr high_link;
		physics::LinkPtr low_link;
		physics::LinkPtr fin_link;

		physics::JointPtr shoulder_joint;
		physics::JointPtr high_joint;
		physics::JointPtr low_joint;
		physics::JointPtr fin_joint;

		linalg::vec<double, 3> final_target;

		linalg::vec<double, 3> speed_target;
		linalg::vec<double, 3> position_target;

		std::vector<physics::JointPtr> joints;

		std::vector<rabbit::htrans3<double>> joint_anchors;
		std::vector<rabbit::htrans3<double>> joint_poses;
		std::vector<rabbit::htrans3<double>> joint_transes;

		std::vector<double> joint_coordinates;
		std::vector<rabbit::screw<double, 3>> joint_local_axes;

		std::vector<rabbit::screw<double, 3>> sensivities;
		std::vector<double> signals;

		std::vector<double> A_matrix_data;

		std::vector<Regulator> regulators;

		void enable_provide_feedback()
		{
			for (auto j : joints) j->SetProvideFeedback(true);
		}

		linalg::vec<double,3> relax_pose;

		LegController(physics::ModelPtr model, int number, linalg::vec<double,3> relax_pose)
		{
			this->relax_pose = relax_pose;

			nos::println("Construct LegController ", number);

			body_link = model->GetLink("body");

			high_link = model->GetLink(nos::format("high_link_{}", number));
			low_link = model->GetLink(nos::format("low_link_{}", number));
			fin_link = model->GetLink(nos::format("fin_link_{}", number));

			shoulder_joint = model->GetJoint(nos::format("shoulder_joint_{}", number));
			high_joint = model->GetJoint(nos::format("high_joint_{}", number));
			low_joint = model->GetJoint(nos::format("low_joint_{}", number));
			fin_joint = model->GetJoint(nos::format("fin_joint_{}", number));

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

		void SpeedControl(linalg::vec<double, 3> vec)
		{
			mode = LegMode::SpeedMode;
			speed_target = vec;
		}

		void PositionControl(linalg::vec<double, 3> vec)
		{
			mode = LegMode::PositionMode;
			position_target = vec;
		}

		bool is_landed()
		{
			auto fin_reaction = rabbit::gazebo_joint_reaction(fin_joint);
			bool ret = fin_reaction.lin.z > 1e-5;
			return ret;
		}

		void serve(double delta)
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
	};

	class TrotController
	{
		BodyController * body_controller;

		std::vector<LegController *> * active_group;
		std::vector<LegController *> * relax_group;

		std::vector<LegController *> group0;
		std::vector<LegController *> group1;


		TrotController(BodyController * body_controller) :
			body_controller()
		{
			active_group = &group0;

			group0.push_back(body_controller->legs[0]);
			group0.push_back(body_controller->legs[2]);
			group0.push_back(body_controller->legs[4]);

			group1.push_back(body_controller->legs[1]);
			group1.push_back(body_controller->legs[3]);
			group1.push_back(body_controller->legs[5]);
		};

		bool evaluate_active_group()
		{

		}

		void serve(double delta)
		{
			evaluate_active_group();

			for (auto l : active_group)
			{
				l->SpeedControl(-body_controller->target_body_speed);
				l->serve(delta);
			}

			for (auto l : relax_group)
			{
				if (stage == 0)
				{
					l->SpeedControl({0, 0, 0});
				}
				else if (stage == 0)
				{
					l->SpeedControl({0, 0, 0});
				}
				else if (stage == 0)
				{
					l->SpeedControl({0, 0, 0});
				}

				l->serve(delta);
			}
		}
	};

	class StepController
	{
		int relax_leg;
		BodyController * body_controller;

		StepController(BodyController * body_controller) :
			body_controller(body_controller)
		{}

		void serve()
		{
			evaluate_active()

			for (int i = 0; i < body_controller->legs.size(); i++)
			{
				if (i != relax_leg)
				{
					l->PositionControl(l->relax_pose 
						+ linalg::vec<double>{10, 0, 0});
				}
				else 
				{
					if (l->is_landed) 
					{
						SpeedControl({0,0,-0.5});
					}
					else 
				}
			}

			l->serve(delta);
		}
	};

	class BodyController
	{
	public:
		std::vector<LegController> legs;
		physics::ModelPtr model;

		rabbit::htrans3<double> body_pose;
		rabbit::htrans3<double> body_target;
		rabbit::screw<double, 3> body_error;

		TrotControler trot_controller;

		BodyController() :
			trot_controller(this)
		{}

		void init(physics::ModelPtr model)
		{
			this->model = model;
			legs.clear();

			for (int i = 0; i < 6; ++i)
				legs.emplace_back(model, i);

			for (auto l : legs) l.enable_provide_feedback();

			legs[0].relax_pose = {-1, 1, 0 };
			legs[1].relax_pose = {-1, 0, 0 };
			legs[2].relax_pose = {-1,-1, 0 };
			legs[3].relax_pose = { 1, 1, 0 };
			legs[4].relax_pose = { 1, 0, 0 };
			legs[5].relax_pose = { 1,-1, 0 };
		}

		void Control(double delta)
		{
			body_pose = rabbit::gazebo_link_pose(model->GetLink("body"));
			body_error = (body_pose.inverse() * body_target).to_screw();

			for (auto l : legs)
			{
				l.SpeedControl({0, 0, 1});
				l.serve(delta);
			}
		};

		void TrotControl(double delta)
		{
			trot_controller.serve();
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

				return;
			}

			for (int i = 0; i < 6; ++i)
			{
				body_controller.legs[i].Control(delta);
			}

			lasttime = curtime;
		}

	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(ModelPush)
}