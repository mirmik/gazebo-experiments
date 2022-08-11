#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/gazebo_client.hh>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>
#include <nos/fprint.h>

void world_stats_cb(const ConstWorldStatisticsPtr &msg)
{
    // pass
    // nos::println(msg->DebugString());
}

void link_stats_cb(const std::string &msg)
{
    // pass
    nos::println(msg);
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
    subscribers.push_back(node->Subscribe("~/hello", link_stats_cb));

    auto topicnamespace = node->GetTopicNamespace();
    nos::println("topicnamespace:", topicnamespace);

    while (true)
    {
        gazebo::common::Time::MSleep(1000);
    }
    return 0;
}