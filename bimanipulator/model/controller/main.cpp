#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <igris/util/string.h>
#include <nos/fprint.h>
#include <ralgo/space/pose3.h>
#include <user_message.pb.h>

class ManipulatorState;
class LinkState
{
    ManipulatorState *parent = nullptr;
    ralgo::pose3<double> pose = {};

public:
    LinkState() = default;

    LinkState(ManipulatorState *parent) : parent(parent) {}

    void set_pose(ralgo::pose3<double> pose)
    {
        this->pose = pose;
    }

    void set_pose(ignition::math::Pose3d pose)
    {
        this->pose = ralgo::pose3<double>(
            {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()},
            {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()});
    }
};

class ModelState
{
    ralgo::pose3<double> pose;
    std::map<std::string, LinkState> links;

public:
    ModelState() {}

    void set_pose(ralgo::pose3<double> pose)
    {
        this->pose = pose;
    }

    void set_pose(ignition::math::Pose3d pose)
    {
        this->pose = ralgo::pose3<double>(
            {pose.Rot().X(), pose.Rot().Y(), pose.Rot().Z(), pose.Rot().W()},
            {pose.Pos().X(), pose.Pos().Y(), pose.Pos().Z()});
    }

    template <typename T> void set_link_pose(std::string link, const T &pose)
    {
        links[link].set_pose(pose);
    }
};

std::map<std::string, ModelState> states;

void world_stats_cb(const ConstWorldStatisticsPtr &msg) {}

ralgo::pose3<double> gazebo_to_ralgo(const gazebo::msgs::Pose &pose)
{
    return ralgo::pose3<double>(
        {pose.orientation().x(),
         pose.orientation().y(),
         pose.orientation().z(),
         pose.orientation().w()},
        {pose.position().x(), pose.position().y(), pose.position().z()});
}

void link_stats_cb(const ConstPosesStampedPtr &msg)
{
    for (int i = 0; i < msg->pose_size(); i++)
    {
        std::string name = msg->pose(i).name();
        std::vector<std::string> parts = igris::split(name, "::");
        if (parts.size() == 1)
        {
            states[parts[0]].set_pose(gazebo_to_ralgo(msg->pose(i)));
        }
        else if (parts.size() == 2)
        {
            states[parts[0]].set_link_pose(parts[1],
                                           gazebo_to_ralgo(msg->pose(i)));
        }
        else
        {
            nos::println("Unknown link name: ", name);
        }
    }
}

void joint_stats_cb(
    const boost::shared_ptr<user_messages::msgs::JointStateArray const> &msg)
{
    // nos::println(msg->DebugString());
}

std::list<gazebo::transport::SubscriberPtr> subscribers;

int main(int argc, char *argv[])
{
    gazebo::client::setup(argc, argv);
    gazebo::transport::NodePtr node(new gazebo::transport::Node());
    node->Init();

    gazebo::transport::PublisherPtr reset_pub =
        node->Advertise<gazebo::msgs::WorldControl>("~/world_control");

    nos::println("Waiting for connection...");
    reset_pub->WaitForConnection();
    nos::println("Connected!");

    gazebo::msgs::WorldControl w_ctrl;
    w_ctrl.mutable_reset()->set_all(true);
    reset_pub->Publish(w_ctrl);

    subscribers.push_back(node->Subscribe("~/world_stats", world_stats_cb));
    subscribers.push_back(node->Subscribe("~/pose/info", link_stats_cb));
    subscribers.push_back(
        node->Subscribe("~/manip1/joint_info", joint_stats_cb));
    subscribers.push_back(
        node->Subscribe("~/manip2/joint_info", joint_stats_cb));

    auto topicnamespace = node->GetTopicNamespace();
    nos::println("topicnamespace:", topicnamespace);

    // publish to wrench topic
    gazebo::transport::PublisherPtr wrench_pub =
        node->Advertise<gazebo::msgs::Wrench>("~/manip1/link_0/wrench");

    wrench_pub->WaitForConnection();

    gazebo::msgs::Wrench wrenchMsg;
    gazebo::msgs::Set(wrenchMsg.mutable_force(),
                      ignition::math::Vector3d::Zero);
    gazebo::msgs::Set(wrenchMsg.mutable_torque(),
                      ignition::math::Vector3d(100, 100, 100));
    gazebo::msgs::Set(wrenchMsg.mutable_force_offset(),
                      ignition::math::Vector3d::Zero);

    while (true)
    {
        gazebo::common::Time::MSleep(100);
        wrench_pub->Publish(wrenchMsg);
    }
    return 0;
}