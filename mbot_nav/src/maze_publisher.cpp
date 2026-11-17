// Checkpoint 1 submission path (rob550 checkpoint1-maze.png).
// Start at (0,0) facing +x. Grid cell = 0.61 m.
#include <cmath>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "mbot_interfaces/msg/pose2_d_array.hpp"

class MazePublisher : public rclcpp::Node {
public:
    MazePublisher() : Node("maze_publisher") {
        pub_ = create_publisher<mbot_interfaces::msg::Pose2DArray>("/waypoints", rclcpp::QoS(1).transient_local());
        const double s = declare_parameter<double>("cell", 0.61);
        // theta = heading on arrival (direction of the segment just driven) -> no extra turn at corners
        mbot_interfaces::msg::Pose2DArray msg;
        msg.poses = {
            pose(1*s,  0,    0.0),
            pose(1*s, -1*s, -M_PI_2),
            pose(2*s, -1*s,  0.0),
            pose(2*s,  1*s,  M_PI_2),
            pose(3*s,  1*s,  0.0),
            pose(3*s, -1*s, -M_PI_2),
            pose(4*s, -1*s,  0.0),
            pose(4*s,  0,    M_PI_2),
            pose(5*s,  0,    0.0),
        };
        pub_->publish(msg);
        RCLCPP_INFO(get_logger(), "Published checkpoint maze path with %zu poses (cell %.2f m).", msg.poses.size(), s);
    }
private:
    rclcpp::Publisher<mbot_interfaces::msg::Pose2DArray>::SharedPtr pub_;
    static geometry_msgs::msg::Pose2D pose(double x, double y, double th) {
        geometry_msgs::msg::Pose2D p; p.x = x; p.y = y; p.theta = th; return p;
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MazePublisher>());
    rclcpp::shutdown();
    return 0;
}
