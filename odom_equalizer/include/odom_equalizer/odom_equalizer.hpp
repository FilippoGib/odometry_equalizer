#ifndef ODOM_EQUALIZER
#define ODOM_EQUALIZER

#include <math.h>
#include <memory>
#include <unistd.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/duration.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <ignition/transport/Node.hh>
#include <ignition/msgs.hh>
#include <ignition/math/Quaternion.hh>
#include <ignition/math/Vector3.hh>

namespace odom_equalizer {

    class OdomEqualizer : public rclcpp::Node
    {
        // pubs and subs
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odomSub;

        // params
        std::string param_odom_topic;
        std::string param_service_topic;

        // members
        std::shared_ptr<ignition::transport::Node> ign_node;
        nav_msgs::msg::Odometry latest_odom;
        rclcpp::TimerBase::SharedPtr timer;

        public:
            OdomEqualizer();

            void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
            void set_pose();
            void ignition_callback(const ignition::msgs::Boolean &reply, const bool result); // used as placeholder
            void load_parameters();
            void inizialize();

    };

}; // odom_equalizer

#endif