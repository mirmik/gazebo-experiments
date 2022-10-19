#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <igris/util/string.h>
#include <nos/fprint.h>
#include <ralgo/rxsignal/rxpid.h>
#include <ralgo/space/pose3.h>
#include <unordered_map>
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

namespace std
{
    template <typename... TTypes> class hash<std::tuple<TTypes...>>
    {
    private:
        typedef std::tuple<TTypes...> Tuple;

        template <int N> size_t operator()(Tuple value) const
        {
            return 0;
        }

        template <int N, typename THead, typename... TTail>
        size_t operator()(Tuple value) const
        {
            constexpr int Index = N - sizeof...(TTail) - 1;
            return hash<THead>()(std::get<Index>(value)) ^
                   operator()<N, TTail...>(value);
        }

    public:
        size_t operator()(Tuple value) const
        {
            return operator()<sizeof...(TTypes), TTypes...>(value);
        }
    };
}

std::unordered_map<std::tuple<std::string, std::string, int>,
                   rxcpp::subjects::subject<double>>
    joint_position_subjects;

void joint_stats_cb(
    const boost::shared_ptr<user_messages::msgs::JointStateArray const> &msg)
{
    for (int i = 0; i < msg->state_size(); i++)
    {
        for (int j = 0; j < msg->state(i).coord_size(); j++)
        {
            joint_position_subjects[{msg->name(), msg->state(i).name(), j}]
                .get_subscriber()
                .on_next(msg->state(i).coord(j));
        }
    }
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
        node->Advertise<user_messages::msgs::JointTorque>(
            "~/manip1/joint_torque");

    wrench_pub->WaitForConnection();

    {
        user_messages::msgs::JointTorque wrench;
        wrench.set_name("joint1");
        wrench.add_torque(100);
        wrench_pub->Publish(wrench);
    }

    {
        user_messages::msgs::JointTorque wrench;
        wrench.set_name("joint0");
        wrench.add_torque(100);
        wrench_pub->Publish(wrench);
    }

    joint_position_subjects[{"manip1", "joint0", 0}].get_observable().subscribe(
        [](double value) { nos::println("joint0:", value); });

    while (true)
    {
        gazebo::common::Time::MSleep(100);
    }
    return 0;
}