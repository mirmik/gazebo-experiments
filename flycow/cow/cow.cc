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
			double time = curtime - starttime;			

			if (!inited)
			{
				nos::reset_terminal();
				nos::println("Init plugin for", this->model->GetName());
				inited = 1;

				//auto joint0 = model->GetJoint("joint0");
				//joint0->SetProvideFeedback(true);

				lasttime = curtime;

				return;
			}

			auto link = model->GetLink("cow0");

			Control(link, delta);
			lasttime = curtime;
		}

		rabbit::screw<double,3> errpos_integral = {{0,0,0},{0,0,0}};
		rabbit::screw<double,3> errspd_integral = {{0,0,0},{0,0,0}};
		void Control(gazebo::physics::LinkPtr link, double delta)
		{
			PRINT(delta);

			auto rot0 = link->WorldCoGPose().Rot();
			auto pos0 = link->WorldCoGPose().Pos();

			auto vel0 = link->WorldLinearVel();
			auto anv0 = link->WorldAngularVel();

			rabbit::htrans3<double> target_pos = 
			{
				{ 0, 0, 0, 1 },
				{ 1, 1, 1 }
			};

			rabbit::htrans3<double> pos = 
			{
				{ rot0.X(), rot0.Y(), rot0.Z(), rot0.W() },
				{ pos0.X(), pos0.Y(), pos0.Z() }
			};

			rabbit::screw<double,3> spd = 
			{
				{ anv0.X(), anv0.Y(), anv0.Z() },
				{ vel0.X(), vel0.Y(), vel0.Z() }
			};

			PRINT(pos);
			PRINT(spd);

			rabbit::htrans3<double> errpos = 
				pos.inverse() * target_pos;

			auto errpos_screw = errpos.to_screw(); 	

			errpos_integral += errpos_screw * delta;	

			double pos_kp = 0.0001;
			double pos_ki = 0.0001;

			auto speed_target = 
				pos_kp * errpos_screw +
				pos_ki * errpos_integral;

			auto errspd_screw = speed_target - spd;
			errspd_integral += errspd_screw * delta;	

			PRINT(errpos_screw);
			PRINT(errspd_screw);

			double spd_kp = 0.0001;
			double spd_ki = 0.0001;

			auto force_target = 
				spd_kp * errspd_screw +
				spd_ki * errspd_integral;

			/*link->SetForce(
				ignition::math::v6::Vector3<double>(
					force_target.lin.x,
					force_target.lin.y,
					force_target.lin.z
				));*/

			PRINT(force_target);
			link->SetTorque(
				ignition::math::v6::Vector3<double>(
					force_target.ang.x,
					force_target.ang.y,
					force_target.ang.z
				));

			exit(0);

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