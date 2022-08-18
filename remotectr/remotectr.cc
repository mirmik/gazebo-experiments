#include <chrono>
#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <nos/fprint.h>
#include <user_message.pb.h>

const std::string remotectr_theme = "/rctr/";

namespace gazebo
{
    class RemoteControlled : public ModelPlugin
    {
        physics::ModelPtr _parent;
        event::ConnectionPtr updateConnection;
        gazebo::transport::PublisherPtr joint_pub;
        gazebo::transport::NodePtr node;
        gazebo::transport::SubscriberPtr joint_torque_subscriber;

        void Load(physics::ModelPtr _parent, sdf::ElementPtr /*_sdf*/) override
        {
            this->_parent = _parent;
            nos::fprintln("Load model: {}", _parent->GetName());
            nos::fprintln("Model has links:");
            for (auto it = _parent->GetLinks().begin();
                 it != _parent->GetLinks().end();
                 ++it)
            {
                auto &link = **it;
                nos::fprintln("\t{}", link.GetName());
            }
            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {
                auto &joint = **it;
                nos::fprintln(
                    "\t{} type: {}", joint.GetName(), joint.GetType());
            }
            this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&RemoteControlled::OnUpdate, this));

            node = transport::NodePtr(new gazebo::transport::Node());
            node->Init();

            std::string world_name = _parent->GetWorld()->Name();
            std::string model_name = _parent->GetName();

            std::string joint_info_topic =
                "/gazebo/" + world_name + "/" + model_name + "/joint_info";
            joint_pub =
                this->node->Advertise<user_messages::msgs::JointStateArray>(
                    joint_info_topic);

            joint_torque_subscriber = node->Subscribe(
                "/gazebo/" + world_name + "/" + model_name + "/joint_torque",
                &RemoteControlled::joint_torque_cb,
                this);
        }

        void joint_torque_cb(
            const boost::shared_ptr<user_messages::msgs::JointTorque const>
                &msg)
        {
            auto name = msg->name();
            auto joint = _parent->GetJoint(name);

            if (joint == nullptr)
            {
                nos::println("Warn: Unknown joint name: ", name);
            }

            for (int i = 0; i < msg->torque_size(); i++)
            {
                auto torque = msg->torque(i);
                joint->SetForce(i, torque);
            }
        }

        void OnUpdate()
        {
            {
                user_messages::msgs::JointStateArray msg;
                msg.set_name(_parent->GetName());
                for (auto it = _parent->GetJoints().begin();
                     it != _parent->GetJoints().end();
                     ++it)
                {
                    user_messages::msgs::JointState *state = msg.add_state();
                    auto &joint = **it;
                    state->set_name(joint.GetName());
                    auto dof = joint.DOF();
                    for (int i = 0; i < dof; ++i)
                    {
                        double position = joint.Position(i);
                        state->add_coord(position);
                    }
                }
                joint_pub->Publish(msg);
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(RemoteControlled)
} // namespace gazebo