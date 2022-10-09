#include <chrono>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <mosquitto_client/mqtt.h>
#include <nos/fprint.h>
#include <user_message.pb.h>

extern mqtt_client *MQTT;
const std::string remotectr_theme = "/rctr/";

class Regulator
{
    double integral;
    double kp = 0, kd = 0, ki = 0;
    double target;
    double ierror = 0;

public:
    Regulator() = default;

    void set_coeffs(double kp, double ki, double kd)
    {
        this->kp = kp;
        this->kd = kd;
        this->ki = ki;
    }

    void set_target(double target)
    {
        this->target = target;
    }

    double update(double pos, double vel, double dt)
    {
        double error = target - pos;
        double derror = -vel;
        ierror += error * dt;
        return kp * error + ki * ierror + kd * derror;
    }
};

namespace gazebo
{
    extern physics::WorldPtr WORLD;

    class RemoteControlled : public ModelPlugin
    {
        physics::ModelPtr _parent;
        event::ConnectionPtr updateConnection;
        gazebo::transport::PublisherPtr joint_pub;
        gazebo::transport::NodePtr node;
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

            node = transport::NodePtr(new gazebo::transport::Node());
            node->Init();

            std::string world_name = _parent->GetWorld()->Name();
            std::string model_name = _parent->GetName();

            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {

                std::string joint_name = (*it)->GetName();
                regulators[joint_name] = Regulator();
                regulators[joint_name].set_coeffs(20, 0, 100);
                regulators[joint_name].set_target(1);

                /*auto &joint = **it;
                std::string joint_name = joint.GetName();
                std::string topic_name =
                    "/" + model_name + "/" + joint_name + "/t";
                auto dof = joint.DOF();*/

                // for (int i = 0; i < dof; i++)
                //{
                /*std::string topic_name_full =
                    topic_name + "/" + std::to_string(i);

                MQTT->subscribe(
                    topic_name_full,
                    [this, &joint, i](const std::string &payload)
                    {
                        auto torque = std::stod(payload);
                        joint.SetForce(i, torque);
                    });*/
                //}
            }
        }

        void OnUpdate()
        {
            // get simulation time
            auto now = WORLD->SimTime();

            if ((now - last_update_time) < gazebo::common::Time(0.01))
                return;

            auto model_name = _parent->GetName();
            std::string modelname = _parent->GetName();

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