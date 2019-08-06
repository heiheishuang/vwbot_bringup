#include <vwbot_bringup/VwbotRosDriver.h>

using namespace std;
using namespace vwpp;

VwbotRosDriver::VwbotRosDriver():
    nh(ros::NodeHandle("~")),
    model("VWBOT_G1"),
    port("/dev/ttyUSB0"),
    baud(115200),
    frame_id("base_link"),
    msg_length(40)
{
    this->node_name = ros::this_node::getName();

    if (nh.hasParam("model"))
    {
        nh.getParam("model", this->model);
        ROS_INFO("%s, use model %s", this->node_name.c_str(), this->model.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default model %s", this->node_name.c_str(), this->model.c_str());
    }


    if (nh.hasParam("port"))
    {
        nh.getParam("port", this->port);
        ROS_INFO("%s, use port %s", this->node_name.c_str(), this->port.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default port %s", this->node_name.c_str(), this->port.c_str());
    }
    
    if (nh.hasParam("baud"))
    {
        nh.getParam("baud", this->baud);
        ROS_INFO("%s, use baud %d", this->node_name.c_str(), this->baud);
    }
    else
    {
        ROS_WARN("%s, use the default baud %d", this->node_name.c_str(), this->baud);
    }

    if (nh.hasParam("msg_length"))
    {
        nh.getParam("msg_length", this->msg_length);
        ROS_INFO("%s, msg_length %d", this->node_name.c_str(), this->msg_length);
    }
    else
    {
        ROS_WARN("%s, use the default msg_length %d", this->node_name.c_str(), this->msg_length);
    }

    this->vwbot_serial_hardware = new VwbotSerialHardware(this->model,
                                                    this->port,
                                                    this->baud,
                                                    this->msg_length);

    if (nh.hasParam("frame_id"))
    {
        nh.getParam("frame_id", this->frame_id);
        ROS_INFO("%s, use frame_id %s", this->node_name.c_str(), this->frame_id.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default frame_id %s", this->node_name.c_str(), this->frame_id.c_str());
    }

    // TODO: Consider add to the initial list.
    std::string cmd_vel_sub_topic_name = "/cmd_vel_unstamped";
    if (nh.hasParam("cmd_vel_topic"))
    {
        nh.getParam("cmd_vel_topic", cmd_vel_sub_topic_name);
        ROS_INFO("%s, use cmd_vel topic name %s", this->node_name.c_str(), cmd_vel_sub_topic_name.c_str());
    }
    else
    {
        ROS_WARN("%s, use the default cmd_vel topic name %s", this->node_name.c_str(), cmd_vel_sub_topic_name.c_str());
    }

    this->cmd_vel_sub = nh.subscribe<geometry_msgs::Twist>(cmd_vel_sub_topic_name, 1, &VwbotRosDriver::cmd_vel_stamped_cb, this);

}



VwbotRosDriver::~VwbotRosDriver()
{
    delete this->vwbot_serial_hardware;
}



void VwbotRosDriver::cmd_vel_stamped_cb(const geometry_msgs::Twist::ConstPtr& msg)
{

    VwbotSerialHardware::Velocity2D vwbot_cmd_vel{};
    vwbot_cmd_vel.x = msg->linear.x;
    vwbot_cmd_vel.y = msg->linear.y;
    vwbot_cmd_vel.yaw = msg->angular.z;

    if (this->vwbot_serial_hardware->sendMessage(vwbot_cmd_vel) != 1)
    {
        ROS_ERROR("%s, Send Message failed!", this->node_name.c_str());
    }

}

