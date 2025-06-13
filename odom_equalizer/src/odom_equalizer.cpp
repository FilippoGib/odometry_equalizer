#include "odom_equalizer.hpp"
#include "gz/math/Vector3.hh"

odom_equalizer::OdomEqualizer::OdomEqualizer() : Node("odom_equalizer")
{
    this->inizialize();
}

nav_msgs::msg::Odometry get_initial_odom()
{
    auto odom = nav_msgs::msg::Odometry();
    ignition::math::Quaterniond correction_quat(
        ignition::math::Vector3d(M_PI_2, 0, 0)  // Roll +90 degrees
    );
    odom.pose.pose.orientation.w = correction_quat.W();
    odom.pose.pose.orientation.x = correction_quat.X();
    odom.pose.pose.orientation.y = correction_quat.Y();
    odom.pose.pose.orientation.z = correction_quat.Z();
    odom.pose.pose.position.x = 0.0;
    odom.pose.pose.position.y = 0.0;
    odom.pose.pose.position.z = -0.30;
    return odom;
}


void odom_equalizer::OdomEqualizer::inizialize()
{
 
    this->load_parameters();

    this->odomSub = this->create_subscription<nav_msgs::msg::Odometry>(this->param_odom_topic, 1, std::bind(&OdomEqualizer::odom_callback, this, std::placeholders::_1));

    this->ign_node = std::make_shared<ignition::transport::Node>();

    // Initialize lates_odom with a default value to avoid the publishing of an uninitialized message
    this->latest_odom = get_initial_odom();

    // create a timer to periodically call set_pose
    this->timer = this->create_wall_timer(std::chrono::milliseconds(100), std::bind(&OdomEqualizer::set_pose, this));

    RCLCPP_INFO(this->get_logger(), "NODE IS INITIALIZED");
}

void odom_equalizer::OdomEqualizer::load_parameters()
{
    this->declare_parameter<std::string>("odom_topic", "/Odometry");
    this->param_odom_topic = this->get_parameter("odom_topic").get_value<std::string>();

    this->declare_parameter<std::string>("service_name", "/world/sim/set_pose");
    this->param_service_topic = this->get_parameter("service_name").get_value<std::string>();

    RCLCPP_INFO(this->get_logger(), "PARAMETERS LOADED");
}

void odom_equalizer::OdomEqualizer::odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "RECEIVED ODOMETRY MESSAGE");
    this->latest_odom = *msg;
}

void odom_equalizer::OdomEqualizer::set_pose()
{
    ignition::msgs::Pose pose_msg;
    pose_msg.set_name("vehicle_blue");

    // Set position
    auto position = pose_msg.mutable_position();
    position->set_x(this->latest_odom.pose.pose.position.x);
    position->set_y(this->latest_odom.pose.pose.position.y);
    position->set_z(this->latest_odom.pose.pose.position.z + 0.8); // this offset is needed, otherwise the vehicle will be partially underground

    // Set orientation
    ignition::math::Quaterniond quat(
        this->latest_odom.pose.pose.orientation.w, 
        this->latest_odom.pose.pose.orientation.x, 
        this->latest_odom.pose.pose.orientation.y, 
        this->latest_odom.pose.pose.orientation.z
    );

    ignition::math::Quaterniond correction_quat(
        ignition::math::Vector3d(-M_PI_2, 0, 0)  // Roll -90 degrees
    );

    quat = quat * correction_quat;

    auto orientation = pose_msg.mutable_orientation();

    orientation->set_x(quat.X());
    orientation->set_y(quat.Y());
    orientation->set_z(quat.Z());
    orientation->set_w(quat.W());

    ignition::msgs::Boolean reply;
    bool call_result = false;
    // Call Ignition service
    bool ok = ign_node->Request<ignition::msgs::Pose, ignition::msgs::Boolean>("/world/sim/set_pose", pose_msg, 100, reply, call_result);

    if (ok && call_result && reply.data()) 
    {
        RCLCPP_INFO(this->get_logger(), "SUCCESS");
    }
    else 
    {
        RCLCPP_INFO(this->get_logger(), "FAILED");
    }
}