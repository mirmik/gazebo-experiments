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

		double spd_T = 0.06;
		double spd_ksi = 3;
		double spd_A = 0.083;

		double pos_T = 1;
		double pos_ksi = 3;

		double pos_kp = 0; 
		double pos_ki = 0; 

		double spd_kp = 0; 
		double spd_ki = 0; 

		physics::JointPtr joint;
		//double force_compensation = 0.1;

		double control_signal = 0;
		double ForceKoeff = 0.1;

		void update_regs() 
		{
			spd_kp = 2.*spd_ksi / spd_T * spd_A;
			spd_ki = 1. / spd_T / spd_T * spd_A;

			pos_kp = 2.*pos_ksi / pos_T;
			pos_ki = 1. / pos_T / pos_T;


			pos_kp = 1.6;
			pos_ki = 0;
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

		LegController(physics::ModelPtr model, int number);

		void SpeedControl(linalg::vec<double, 3> vec);

		void PositionControl(linalg::vec<double, 3> vec);

		bool is_landed();

		void serve(double delta);
	};

	class BodyController;

	class TrotController
	{
	public:
		int stage = 0;

		BodyController * body_controller;

		std::vector<LegController *> * active_group;
		std::vector<LegController *> * relax_group;

		std::vector<LegController *> group0;
		std::vector<LegController *> group1;


		TrotController(BodyController * body_controller);
		bool evaluate_active_group();
		void serve(double delta);
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
		physics::ModelPtr model;

		rabbit::htrans3<double> body_pose;
		rabbit::htrans3<double> body_target;
		rabbit::screw<double, 3> body_error;

		linalg::vec<double,3> target_body_speed;

		TrotController trot_controller;

		BodyController();

		void init(physics::ModelPtr model);

		void Control(double delta);

		void TrotControl(double delta);
	};
}

#endif