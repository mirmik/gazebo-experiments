#include <gazebo/common/common.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <ignition/math/Vector3.hh>
#include <nos/fprint.h>

const std::string remotectr_theme = "/rctr/";

namespace gazebo
{
    extern physics::WorldPtr WORLD;

    class RemoteControlled : public ModelPlugin
    {
        physics::ModelPtr _parent;
        event::ConnectionPtr updateConnection;

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
        }

        /*void publish(const std::string theme,
                     const ignition::math::Pose3d &pose)
        {
            auto lin = pose.Pos();
            auto ang = pose.Rot();
            auto msg = nos::format("{{'lin':[{},{},{}],'ang':[{},{},{},{}]}",
                                   lin.X(),
                                   lin.Y(),
                                   lin.Z(),
                                   ang.X(),
                                   ang.Y(),
                                   ang.Z(),
                                   ang.W());
            crow::publish(theme, msg);
        }*/

        void OnUpdate()
        {
            auto pose = _parent->WorldPose();
            // publish(remotectr_theme + _parent->GetName() + "/world_pose",
            // pose);

            for (auto it = _parent->GetLinks().begin();
                 it != _parent->GetLinks().end();
                 ++it)
            {
                auto &link = **it;
                auto pose = link.WorldPose();
                /*publish(remotectr_theme + _parent->GetName() + "/" +
                            link.GetName() + "/world_pose",
                        pose);*/
            }

            for (auto it = _parent->GetJoints().begin();
                 it != _parent->GetJoints().end();
                 ++it)
            {
                auto &joint = **it;
                auto dof = joint.DOF();
                for (int i = 0; i < dof; ++i)
                {
                    double position = joint.Position(i);
                }
            }
        }
    };

    GZ_REGISTER_MODEL_PLUGIN(RemoteControlled)
} // namespace gazebo