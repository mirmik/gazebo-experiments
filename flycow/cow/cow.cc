#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <igris/util/iteration_counter.h>

#include <nos/print.h>
#include <nos/fprint.h>
#include <nos/terminal.h>

#include <linalg/linalg.h>
#include <ralgo/linalg/backpack.h>
#include <rabbit/space/htrans3.h>
#include <rabbit/space/screw.h>

#include <igris/math.h>

namespace gazebo
{
	extern physics::WorldPtr WORLD;

	class CowModel : public ModelPlugin
	{
	private:
		double ForceKoeff = 0.001;
		// Pointer to the model
		physics::ModelPtr model;

		// Pointer to the update event connection
		event::ConnectionPtr updateConnection;

		int inited = 0;

		double time;
		double lasttime;
		double starttime;
		double delta;

	public:

		double evaltime()
		{
			auto t = WORLD->SimTime();
			return (double)t.sec + (double)t.nsec / 1000000000.;
		}

		void Reset()
		{
			nos::println("RESET");
			//errpos_integral = {{0, 0, 0}, {0, 0, 0}};
			//errspd_integral = {{0, 0, 0}, {0, 0, 0}};
			inited = 0;
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			nos::println("Load manipulator");
			// Store the pointer to the model
			this->model = _parent;

			// Listen to the update event. This event is broadcast every
			// simulation iteration.
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&CowModel::OnUpdate, this));

			Reset();
		}

		// Called by the world update start event
	public:

		//linalg::vec<double,2> position_integral {};
		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			time = curtime - starttime;

			if (!inited)
			{
				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;

				//auto joint0 = model->GetJoint("joint0");
				//joint0->SetProvideFeedback(true);
				starttime = curtime;
				lasttime = curtime;

				auto link = model->GetLink("cow0");

				//link->SetWorldPose (ignition::math::v6::Pose3(
				  //                      ignition::math::Vector3<double>(0, 0, 0),
				    //                    ignition::math::Quaternion<double>(1, 0, 0, 0))
				      //             );

				return;
			}

			auto link = model->GetLink("cow0");

			Control(link, delta);
			lasttime = curtime;
		}

		//rabbit::screw<double, 3> errpos_integral = {{0, 0, 0}, {0, 0, 0}};
		//rabbit::screw<double, 3> errspd_integral = {{0, 0, 0}, {0, 0, 0}};
		void Control(gazebo::physics::LinkPtr link, double delta)
		{
			auto rot0 = link->WorldPose().Rot();
			auto pos0 = link->WorldPose().Pos();
			auto irot0 = link->WorldPose().Inverse().Rot();
			auto ipos0 = link->WorldPose().Inverse().Pos();

			auto vel0 = link->WorldLinearVel();
			auto anv0 = link->WorldAngularVel();

			rabbit::htrans3<double> target_pos =
			{
				linalg::rotation_quat<double>(
				linalg::normalize<double, 3>({0, 0, 1}), 0),
				{ 0, 0, 0 }
			};

			if (time > 5)
				target_pos =

				    rabbit::htrans3<double>
			{
				linalg::rotation_quat<double>(
				linalg::normalize<double, 3>({1, 0, 0}), M_PI / 2),
				{ 0, 0, 0 }
			}

			*

			rabbit::htrans3<double>
			{
				linalg::rotation_quat<double>(
				linalg::normalize<double, 3>({0, 0, 1}), M_PI / 2),
				{ 0, 0, 0 }
			}
			;


			if (time > 10)
				target_pos =

				    rabbit::htrans3<double>
			{
				linalg::rotation_quat<double>(
				linalg::normalize<double, 3>({1, 0, 0}), M_PI/2),
				{ 0, 0, 0 }
			};




			rabbit::htrans3<double> pos =
			{
				{ rot0.X(), rot0.Y(), rot0.Z(), rot0.W() },
				{ pos0.X(), pos0.Y(), pos0.Z() }
			};

			rabbit::htrans3<double> ipos =
			{
				{ irot0.X(), irot0.Y(), irot0.Z(), irot0.W() },
				{ ipos0.X(), ipos0.Y(), ipos0.Z() }
			};

			PRINT(pos);
			PRINT(target_pos);

			rabbit::screw<double, 3> spd =
			{
				{ anv0.X(), anv0.Y(), anv0.Z() },
				{ vel0.X(), vel0.Y(), vel0.Z() }
			};

			PRINT(spd);

			rabbit::htrans3<double> errpos =
			    pos.inverse() * target_pos;
			
			
			auto errpos_screw = errpos.to_screw();
			errpos_screw = errpos_screw.rotate_by(pos);

			PRINT(errpos_screw);
			//errpos_integral += errpos_screw * delta;

			/*double pos_T = 5;
			double pos_ksi = 2;

			double pos_kp = 2.*pos_ksi / pos_T;
			double pos_ki = 1. / pos_T / pos_T;

			rabbit::screw<double,3> speed_target = {{0,0,0}, {0,0,0}};
			*/
			/*speed_target = {{0, 5, 0}, {0,0,0}};
			if (time > 7)
				speed_target = {{5, 0, 0}, {0,0,0}};*/

			auto errspd_screw = - spd;
			PRINT(errspd_screw);

			/*errspd_integral += errspd_screw * delta;

			double spd_T = 1;
			double spd_ksi = 0.75;
			double spd_A = 2;

			double spd_kp = 2.*spd_ksi / spd_T * spd_A;
			double spd_ki = 1. / spd_T / spd_T * spd_A;*/

			auto force_target =
			    1 * errpos_screw +
			    2 * errspd_screw
			    //0.1 * errpos_integral
			    ;

			//spd_kp * errspd_screw;
			//spd_ki * errspd_integral;

			//force_target = {{0.1,0,0},{0,0,0}};


			//PRINT(errpos_screw);
			//PRINT(errspd_screw);
			//PRINT(force_target);
			//do_after_iteration(4)
			//	exit(0);

			link->SetForce(
			    ignition::math::v6::Vector3<double>(
			        force_target.lin.x,
			        force_target.lin.y,
			        force_target.lin.z
			    ));

			link->SetTorque(
			    ignition::math::v6::Vector3<double>(
			        force_target.ang.x,
			        force_target.ang.y,
			        force_target.ang.z
			    ));
			/*link->SetForce(
				ignition::math::v6::Vector3<double>(
					0.01,
					0.01,
					0.01
				));
			link->SetTorque(
				ignition::math::v6::Vector3<double>(
					0.01,
					0.01,
					0.01
				));*/
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(CowModel)
}