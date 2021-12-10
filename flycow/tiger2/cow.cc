#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <igris/util/iteration_counter.h>

#include <nos/print.h>
#include <nos/fprint.h>
#include <nos/terminal.h>

#include <ralgo/linalg/linalg.h>
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
			inited = 0;
			errspd_integral = {};
			errpos_integral = {};
		}

		void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/)
		{
			this->model = _parent;
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
			                             std::bind(&CowModel::OnUpdate, this));

			Reset();
		}

	public:

		void OnUpdate()
		{
			double curtime = evaltime();
			delta = curtime - lasttime;
			time = curtime - starttime;

			model->GetLink("vis0")->SetWorldPose (ignition::math::v6::Pose3(
			        ignition::math::Vector3<double>(target_pos.lin.x, target_pos.lin.y, target_pos.lin.z),
			        ignition::math::Quaternion<double>(1, 0, 0, 0))
			                                     );
			model->GetLink("vis0")->SetLinearVel(
					{0,0,0}
				);


			if (!inited)
			{
				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;

				starttime = curtime;
				lasttime = curtime;

				auto link = model->GetLink("cow0");
				return;
			}

			auto link = model->GetLink("cow0");

			Control(link, delta);
			if (evaltime() < curtime)
				curtime = evaltime();
			lasttime = curtime;
		}

		rabbit::htrans3<double> target_pos;
		rabbit::screw<double, 3> errpos_integral = {{0, 0, 0}, {0, 0, 0}};
		rabbit::screw<double, 3> errspd_integral = {{0, 0, 0}, {0, 0, 0}};
		void Control(gazebo::physics::LinkPtr link, double delta)
		{
			auto rot0 = link->WorldPose().Rot();
			auto pos0 = link->WorldPose().Pos();
			auto irot0 = link->WorldPose().Inverse().Rot();
			auto ipos0 = link->WorldPose().Inverse().Pos();

			auto vel0 = link->WorldLinearVel();
			auto anv0 = link->WorldAngularVel();

			target_pos =
			{
				linalg::rotation_quat<double>(
				linalg::normalize<double, 3>({0, 0, 1}), 0),
				{ 0, 0, 0 }
			};



			if (time < 5)
			{
				target_pos =
				{
					{ 0, 0, 0, 1},
					{ 0, time, 0 }
				};
			}
			else if (time < 10)
			{
				target_pos =
				{
					{ 0, 0, 0, 1},
					{ time - 5, 5, 0 }
				};
			}
			else if (time < 15)
			{
				target_pos =
				{
					{ 0, 0, 0, 1},
					{ 5, 5 - (time - 10), 0 }
				};
			}
			else if (time < 20)
			{
				target_pos =
				{
					{ 0, 0, 0, 1},
					{ 5 - (time - 15), 0, 0 }
				};
			}
			else
			{
				starttime = evaltime();
				delta = 0;
				return;
			}

			/*target_pos =
			{
				{ 0, 0, 0, 1},
				{ 0, 0, 0 }
			};*/


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

			rabbit::screw<double, 3> spd =
			{
				{ anv0.X(), anv0.Y(), anv0.Z() },
				{ vel0.X(), vel0.Y(), vel0.Z() }
			};


			rabbit::htrans3<double> errpos =
			    pos.inverse() * target_pos;

			auto errpos_screw = errpos.to_screw();
			errpos_screw = errpos_screw.rotate_by(pos);
			errpos_integral += errpos_screw * delta;

			double pos_T = 2;
			double pos_ksi = 4;

			double pos_kp = 2.*pos_ksi / pos_T;
			double pos_ki = 1. / pos_T / pos_T;

			rabbit::screw<double, 3> speed_target =
			    //pos_ki * errpos_integral +
			    pos_kp * errpos_screw;

			//PRINT(errpos_integral);

			auto errspd_screw = speed_target - spd;

			//errspd_integral += errspd_screw * delta;

			double spd_T = 1;
			double spd_ksi = 0.75 * 2;
			double spd_A = 2;

			double spd_kp = 2.*spd_ksi / spd_T * spd_A;
			double spd_ki = 1. / spd_T / spd_T * spd_A;

			auto marshal = pos.zdir();

			auto spdang = rabbit::screw<double, 3>(spd.ang, {});
			auto spdlin = rabbit::screw<double, 3>({}, spd.lin);

			auto force_target =
			    errpos_screw * 1 +
			    -spdlin * 1.5
			    + rabbit::screw<double, 3>({}, {0, 0, errpos_integral.lin.z}) * 0.2
			    ;


			PRINT(force_target);

			auto zdir_force_target_ang = linalg::dot(force_target.ang, marshal) * marshal;
			auto xydir_force_target_ang = force_target.ang - zdir_force_target_ang;

			auto zdir_force_target_lin = linalg::dot(force_target.lin, marshal) * marshal;
			auto xydir_force_target_lin = force_target.lin - zdir_force_target_lin;

			auto normal = -linalg::cross(xydir_force_target_lin, marshal);
			PRINT(xydir_force_target_lin);
			PRINT(normal);

			auto xysignal =
			    normal * 10
			    - spd.ang * 8;

			bool enable_manvour = false;
			if (linalg::length(errpos_screw.lin) < 0.5) {
				enable_manvour = true;
			}
			

			if (linalg::length(xydir_force_target_lin) > 0.01)
				xydir_force_target_lin = xydir_force_target_lin / linalg::length(xydir_force_target_lin) * 0.1;

			auto force =
			    ignition::math::v6::Vector3<double>(
			        zdir_force_target_lin.x,
			        zdir_force_target_lin.y,
			        zdir_force_target_lin.z
			    );

			if (enable_manvour)
				force = force +
				        ignition::math::v6::Vector3<double>(
				            xydir_force_target_lin.x,
				            xydir_force_target_lin.y,
				            xydir_force_target_lin.z
				        );

			link->SetForce(
			    force
			);

			link->SetTorque(
			    ignition::math::v6::Vector3<double>(
			        xysignal.x,
			        xysignal.y,
			        xysignal.z
			    ));
		}
	};

	// Register this plugin with the simulator
	GZ_REGISTER_MODEL_PLUGIN(CowModel)
}