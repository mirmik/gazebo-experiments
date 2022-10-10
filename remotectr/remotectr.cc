#include <chrono>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <mosquitto_client/mqtt.h>
#include <nos/fprint.h>
#include <nos/util/string.h>
#include <user_message.pb.h>

extern mqtt_client *MQTT;
const std::string remotectr_theme = "/rctr/";

class Regulator
{
    double integral;
    double kp = 0, kd = 0, ki = 0;
    double target;
    double _ierror = 0;
    double ierror_max = 0;
    double ierror_min = 0;

public:
    Regulator() = default;

    void set_coeffs(double kp, double ki, double kd)
    {
        this->kp = kp;
        this->kd = kd;
        this->ki = ki;
    }
    void set_coeffs(double kp, double ki, double kd, double imin, double imax)
    {
        this->kp = kp;
        this->kd = kd;
        this->ki = ki;
        this->ierror_max = imax;
        this->ierror_min = imin;
    }

    void set_ierror(double val)
    {
        _ierror = val;
    }

    void set_target(double target)
    {
        this->target = target;
    }

    double ierror()
    {
        return _ierror;
    }

    double update(double pos, double vel, double dt)
    {
        double error = target - pos;
        double derror = -vel;
        _ierror += error * dt;
        if (_ierror > ierror_max)
            _ierror = ierror_max;
        if (_ierror < ierror_min)
            _ierror = ierror_min;
        return kp * error + ki * _ierror + kd * derror;
    }
};

namespace gazebo
{
    extern physics::WorldPtr WORLD;

    class RemoteControlled : public ModelPlugin
    {
        std::mutex mutex;
        physics::ModelPtr _parent;
        event::ConnectionPtr updateConnection;
        event::ConnectionPtr resetConnection;
        gazebo::transport::SubscriberPtr joint_torque_subscriber;
        gazebo::common::Time last_update_time;

        std::map<std::string, Regulator> regulators;

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override
        {
            this->_parent = _parent;
            nos::fprintln("RemoteCtr plugin loaded. model : {}",
                          _parent->GetName());

            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RemoteControlled::OnUpdate, this));

            this->resetConnection = event::Events::ConnectWorldReset(
                std::bind(&RemoteControlled::OnReset, this));

            std::string world_name = _parent->GetWorld()->Name();
            std::string model_name = _parent->GetName();

            restart_regulators();
            update_simulation_time();

            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {
                auto &joint = **it;

                MQTT->subscribe("/" + model_name + "/" + joint.GetName() +
                                    "/regulator",
                                [this, &joint](const std::string &msg)
                                {
                                    std::lock_guard<std::mutex> lock(mutex);
                                    auto coeffs = nos::split(msg, ',');
                                    regulators[joint.GetName()].set_coeffs(
                                        std::stod(coeffs[0]),
                                        std::stod(coeffs[1]),
                                        std::stod(coeffs[2]),
                                        std::stod(coeffs[3]),
                                        std::stod(coeffs[4]));
                                });

                MQTT->subscribe(
                    "/" + model_name + "/" + joint.GetName() + "/target",
                    [this, &joint](const std::string &msg)
                    {
                        std::lock_guard<std::mutex> lock(mutex);
                        regulators[joint.GetName()].set_target(std::stod(msg));
                    });
            }
        }

        void restart_regulators()
        {
            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {

                std::string joint_name = (*it)->GetName();
                regulators[joint_name] = Regulator();
                regulators[joint_name].set_ierror(0);
                regulators[joint_name].set_coeffs(0, 0, 0);
                regulators[joint_name].set_target(0);
            }
        }

        void update_simulation_time()
        {
            last_update_time = _parent->GetWorld()->SimTime();
        }

        void OnReset()
        {
            std::lock_guard<std::mutex> lock(mutex);
            restart_regulators();
            update_simulation_time();
            nos::println("restart subscriber");
        }

        void OnUpdate()
        {
            std::lock_guard<std::mutex> lock(mutex);
            // get simulation time
            auto now = WORLD->SimTime();

            if ((now - last_update_time) < gazebo::common::Time(0.01))
                return;

            auto model_name = _parent->GetName();
            std::string modelname = _parent->GetName();

            // pose
            auto pose = _parent->WorldPose();
            MQTT->publish("/" + model_name + "/pose",
                          std::to_string(pose.Pos().X()) + "," +
                              std::to_string(pose.Pos().Y()) + "," +
                              std::to_string(pose.Pos().Z()) + "," +
                              std::to_string(pose.Rot().W()) + "," +
                              std::to_string(pose.Rot().X()) + "," +
                              std::to_string(pose.Rot().Y()) + "," +
                              std::to_string(pose.Rot().Z()));

            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {
                auto &joint = **it;
                std::string jointname = joint.GetName();
                auto dof = joint.DOF();

                for (int i = 0; i < dof; ++i)
                {

                    double position = joint.Position(i);
                    double velocity = joint.GetVelocity(i);

                    MQTT->publish("/" + modelname + "/" + jointname + "/p/" +
                                      std::to_string(i),
                                  std::to_string(position));

                    MQTT->publish("/" + modelname + "/" + jointname + "/v/" +
                                      std::to_string(i),
                                  std::to_string(velocity));

                    MQTT->publish(
                        "/" + modelname + "/" + jointname + "/ie/" +
                            std::to_string(i),
                        std::to_string(regulators[jointname].ierror()));
                }

                // joint type
                if (joint.HasType(physics::Base::HINGE_JOINT))
                {
                    auto &reg = regulators[jointname];
                    double torque =
                        reg.update(joint.Position(0),
                                   joint.GetVelocity(0),
                                   (now - last_update_time).Double());
                    joint.SetForce(0, torque);
                    // nos::println(torque);
                }
            }
            last_update_time = now;
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(RemoteControlled)
} // namespace gazebo