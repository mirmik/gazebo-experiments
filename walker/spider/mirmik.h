#ifndef MIRMIK_H
#define MIRMIK_H

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
	class BodyController;

	class Regulator
	{
	public:
		bool speed2_loop_enabled = true;
		bool position_loop_enabled = true;

		double speed_error = 0;
		double position_error = 0;

		double speed_target;
		//double position_target = 45. / 180. * 3.14;
		double position_target;

		double speed_integral = 0;
		double position_integral = 0;

		double speed2_target = 0;

		double spd_T = 0.1;
		double spd_ksi = 1;
		double spd_A = 1.33;

		double pos_T = 0.25;
		double pos_ksi = 1;

		double pos_kp = 0;
		double pos_ki = 0;

		double speed_integral_limit = 6;
		double position_integral_limit = 6;


		double spd_kp = 0;
		double spd_ki = 0;

		physics::JointPtr joint;

		double control_signal = 0;
		double ForceKoeff = 0.003 * 0;

		void update_regs()
		{
			spd_kp = 2.*spd_ksi / spd_T * spd_A;
			spd_ki = 1. / spd_T / spd_T * spd_A;

			pos_kp = 2.*pos_ksi / pos_T;
			pos_ki = 1. / pos_T / pos_T;


			pos_kp = 1.6;
			pos_ki = 0;

			speed_integral = 0;
			position_integral = 0;
		}

		Regulator(physics::JointPtr joint);

		void reset();

		void Control(double delta);
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
		BodyController * body_controller;

		physics::LinkPtr body_link;

		physics::LinkPtr shoulder_link;
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

		rabbit::htrans3<double> final_link_local_pose;

		rabbit::htrans3<double> relative_shoulder_pose();
		rabbit::htrans3<double> relative_output_pose();

		void enable_provide_feedback()
		{
			//for (auto j : joints) j->SetProvideFeedback(true);
			fin_joint->SetProvideFeedback(true);
		}

		linalg::vec<double, 3> relax_pose;

		LegController(physics::ModelPtr model, BodyController * body_controller, int number);

		void SpeedControl(linalg::vec<double, 3> vec);

		void PositionControl(linalg::vec<double, 3> vec);

		bool is_landed();

		void serve(double delta);

		rabbit::screw<double, 3> reaction() 
		{
			return rabbit::gazebo_joint_reaction(fin_joint);
		}
	};

	class TrotController
	{
	public:
		//int stage = 0;
		bool relax_end_step = false;

		BodyController * body_controller;

		std::vector<LegController *> active_group;
		std::vector<LegController *> relax_group;

		std::vector<LegController *> group0;
		std::vector<LegController *> group1;


		TrotController(BodyController * body_controller);
		void serve(double delta);

		void change_active_group();
	};

	class StepController
	{
	public:
		int relax_leg;
		BodyController * body_controller;

		StepController(BodyController * body_controller);
		void evaluate_active();
		void serve(double delta);
	};

	class BodyController
	{
	public:
		std::vector<LegController> legs;
		std::vector<LegController*> legs_ptrs;

		std::vector<LegController*> forward_legs;
		std::vector<LegController*> middle_legs;
		std::vector<LegController*> backward_legs;

		physics::ModelPtr model;

		rabbit::htrans3<double> body_pose;
		rabbit::htrans3<double> body_target;
		rabbit::screw<double, 3> body_error;
		rabbit::screw<double, 3> body_error_integral;

		rabbit::screw<double, 3> body_speed_target;

		TrotController trot_controller;

		BodyController();

		void init(physics::ModelPtr model);

		void serve(double delta);

		void TrotControl(double delta);

		void body_serve_with_group(
		    const std::vector<LegController *>& legs, 
		    double delta);

		void relax_movement_with_group(
		    const std::vector<LegController *>& legs, 
		    double delta,
		    double level,
		    linalg::vec<double, 3> speed);

	};

	class ArmsController 
	{
		BodyController * body_controller;
	public:
		ArmsController(BodyController * body_controller)
			: body_controller(body_controller)
		{}
	};

	class FourLegStep 
	{
		BodyController * body_controller;
	public:
		FourLegStep(BodyController * body_controller)
			: body_controller(body_controller)
		{}
	};
}

#endif