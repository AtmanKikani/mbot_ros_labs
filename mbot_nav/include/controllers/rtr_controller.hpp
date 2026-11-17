#pragma once

#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose2_d.hpp>
#include <vector>
#include "utils.hpp"  // wrapToPi

// RTR (Rotate-Translate-Rotate) PID controller.
// Follows a queue of 2D waypoints. Call computeVelocityCommands() at 20 Hz (every 50 ms).
class RTRController
{
public:
    RTRController() = default;

    // Load new waypoints; skips any already within reach of (current_x, current_y)
    void setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                 double current_x, double current_y);

    // Compute and return cmd_vel for the current goal given the robot's current pose.
    // Returns zero twist when there is no active plan.
    geometry_msgs::msg::Twist computeVelocityCommands(double x, double y, double theta);

    // Speed limits (set from controller_node parameters)
    void setLimits(double max_lin, double max_ang) { max_lin_ = max_lin; max_ang_ = max_ang; }
    void setGains(double kp_lin, double kp_ang) { Kp_lin = kp_lin; Kp_ang = kp_ang; }
    void setAccel(double lin_accel, double ang_accel) { max_lin_accel_ = lin_accel; max_ang_accel_ = ang_accel; }
    void setDecel(double lin_decel, double ang_decel) { max_lin_decel_ = lin_decel; max_ang_decel_ = ang_decel; }

private:
    double goal_x_{0}, goal_y_{0}, goal_theta_{0};
    bool   goal_received_{false};
    std::vector<geometry_msgs::msg::Pose2D> goal_queue_;

    // Explicit RTR state so small odometry noise near a goal can't bounce us between phases
    enum class State { ROTATE_TO_TARGET, TRANSLATE, ROTATE_TO_FINAL };
    State state_{State::ROTATE_TO_TARGET};

    double max_lin_{0.3}, max_ang_{1.0};
    double last_lin_cmd_{0.0}, last_ang_cmd_{0.0};

    static constexpr double dist_thresh_     = 0.02;
    static constexpr double angle_thresh_    = 0.02;
    static constexpr double dt_              = 0.05;   // 50 ms control period
    static constexpr double integral_limit_  = 1.0;
    static constexpr double realign_thresh_  = 0.35;  // rad: heading error that aborts TRANSLATE back to ROTATE
    static constexpr double heading_lock_dist_ = 0.10; // m: stop steering toward goal point when this close
    static constexpr double min_lin_         = 0.04;  // m/s: floor so we don't stall short of the goal
    static constexpr double min_ang_         = 0.15;  // rad/s: floor so we don't stall short of the angle
    double max_lin_accel_{0.6};   // m/s^2 slew limit (reduces wheel slip)
    double max_ang_accel_{3.0};   // rad/s^2 slew limit
    double max_lin_decel_{0.6};   // m/s^2 braking limit (prevents tipping)
    double max_ang_decel_{3.0};   // rad/s^2 braking limit

    // PID gains
    double Kp_lin{1.0}; static constexpr double Ki_lin = 0.0, Kd_lin = 0.01;
    double Kp_ang{2.0}; static constexpr double Ki_ang = 0.0, Kd_ang = 0.05;

    // PID state
    double lin_error_sum_{0}, lin_error_last_{0};
    double ang_error_sum_{0}, ang_error_last_{0};
    
    void resetPID();
    double linearPID(double error);
    double angularPID(double error);
    void loadNextGoal();
};
