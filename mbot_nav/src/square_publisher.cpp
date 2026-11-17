#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class SquarePublisher : public rclcpp::Node {
public:
    SquarePublisher() : Node("square_publisher") {
        pub_ = this->create_publisher<mbot_interfaces::msg::Pose2DArray>(
            "/waypoints", rclcpp::QoS(1).transient_local());

        // Publish once — transient_local QoS delivers the cached message to any late-connecting subscriber
        const int loops = this->declare_parameter<int>("loops", 1);
        const double side = this->declare_parameter<double>("side", 1.0);

        mbot_interfaces::msg::Pose2DArray goal_array;
        for (int i = 0; i < loops; ++i) {
            goal_array.poses.push_back(pose(side, 0.0, 0.0));
            goal_array.poses.push_back(pose(side, side, M_PI_2));
            goal_array.poses.push_back(pose(0.0, side, M_PI));
            goal_array.poses.push_back(pose(0.0, 0.0, 0.0));
        }
        pub_->publish(goal_array);
        RCLCPP_INFO(this->get_logger(), "Published square trajectory (%d loops) with %zu poses.", loops, goal_array.poses.size());
    }

private:
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr pub_;

    static geometry_msgs::msg::Pose2D pose(double x, double y, double theta) {
        geometry_msgs::msg::Pose2D p;
        p.x = x;
        p.y = y;
        p.theta = theta;
        return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<SquarePublisher>());
    rclcpp::shutdown();
    return 0;
}