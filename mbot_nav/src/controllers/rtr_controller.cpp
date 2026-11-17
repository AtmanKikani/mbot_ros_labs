#include "controllers/rtr_controller.hpp"
#include <rclcpp/rclcpp.hpp>
#include <algorithm>  // std::clamp
#include <cmath>

void RTRController::setPlan(const std::vector<geometry_msgs::msg::Pose2D>& waypoints,
                            double current_x, double current_y)
{
    if (waypoints.empty()) return;

    goal_queue_.clear();
    for (const auto& p : waypoints) goal_queue_.push_back(p);

    // Skip waypoints already within reach
    while (!goal_queue_.empty()) {
        const auto& front = goal_queue_.front();
        double dx = front.x - current_x;
        double dy = front.y - current_y;
        if (std::sqrt(dx*dx + dy*dy) < dist_thresh_) {
            goal_queue_.erase(goal_queue_.begin());
        } else {
            break;
        }
    }

    state_ = State::ROTATE_TO_TARGET;
    resetPID();

    if (!goal_queue_.empty()) {
        goal_x_       = goal_queue_.front().x;
        goal_y_       = goal_queue_.front().y;
        goal_theta_   = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        goal_received_ = true;
    } else {
        goal_received_ = false;
    }
}

// PID on distance-to-go (m) -> forward speed (m/s)
double RTRController::linearPID(double error)
{
    if (std::isnan(lin_error_last_)) lin_error_last_ = error;  // no D kick after reset
    lin_error_sum_ = std::clamp(lin_error_sum_ + error * dt_, -integral_limit_, integral_limit_);
    double d = (error - lin_error_last_) / dt_;
    lin_error_last_ = error;
    return Kp_lin * error + Ki_lin * lin_error_sum_ + Kd_lin * d;
}

// PID on heading error (rad) -> yaw rate (rad/s)
double RTRController::angularPID(double error)
{
    if (std::isnan(ang_error_last_)) ang_error_last_ = error;  // no D kick after reset
    ang_error_sum_ = std::clamp(ang_error_sum_ + error * dt_, -integral_limit_, integral_limit_);
    double d = (error - ang_error_last_) / dt_;
    ang_error_last_ = error;
    return Kp_ang * error + Ki_ang * ang_error_sum_ + Kd_ang * d;
}

geometry_msgs::msg::Twist RTRController::computeVelocityCommands(double x, double y, double theta)
{
    geometry_msgs::msg::Twist cmd;
    if (!goal_received_) {
        last_lin_cmd_ = last_ang_cmd_ = 0.0;
        return cmd;
    }

    double dx           = goal_x_ - x;
    double dy           = goal_y_ - y;
    double target_angle = std::atan2(dy, dx);
    double dist_error   = std::sqrt(dx*dx + dy*dy);
    double angle_error  = wrapToPi(target_angle - theta);

    double v = 0.0, w = 0.0;

    // Already at the goal position -> go straight to final rotation
    if (state_ != State::ROTATE_TO_FINAL && dist_error < dist_thresh_) {
        state_ = State::ROTATE_TO_FINAL;
        resetPID();
    }

    switch (state_) {
    case State::ROTATE_TO_TARGET: {
        // STATE 1: rotate in place to face the goal point
        if (std::fabs(angle_error) < angle_thresh_) {
            state_ = State::TRANSLATE;
            resetPID();
        } else {
            w = angularPID(angle_error);
            w = std::copysign(std::min(std::fabs(w), std::sqrt(2.0 * max_ang_decel_ * std::fabs(angle_error))), w);
            if (std::fabs(w) < min_ang_) w = std::copysign(min_ang_, angle_error);
        }
        break;
    }
    case State::TRANSLATE: {
        // STATE 2: drive forward with heading correction
        // Distance-to-go measured along our heading so we stop (not reverse) if we overshoot
        double along = dist_error * std::cos(angle_error);
        if (along < dist_thresh_) {
            state_ = State::ROTATE_TO_FINAL;
            resetPID();
            break;
        }
        if (std::fabs(angle_error) > realign_thresh_ && dist_error > heading_lock_dist_) {
            state_ = State::ROTATE_TO_TARGET;
            resetPID();
            break;
        }
        v = linearPID(along);
        // Braking profile: never faster than we can stop from within the remaining distance
        v = std::min(v, std::sqrt(2.0 * max_lin_decel_ * along));
        if (v < min_lin_) v = min_lin_;
        // Steer toward the goal point; near the goal the bearing gets noisy, so stop steering
        if (dist_error > heading_lock_dist_) {
            w = angularPID(angle_error);
        }
        break;
    }
    case State::ROTATE_TO_FINAL: {
        // STATE 3: rotate to the waypoint's final orientation
        double final_error = wrapToPi(goal_theta_ - theta);
        if (std::fabs(final_error) < angle_thresh_) {
            loadNextGoal();
            state_ = State::ROTATE_TO_TARGET;
            resetPID();
        } else {
            w = angularPID(final_error);
            w = std::copysign(std::min(std::fabs(w), std::sqrt(2.0 * max_ang_decel_ * std::fabs(final_error))), w);
            if (std::fabs(w) < min_ang_) w = std::copysign(min_ang_, final_error);
        }
        break;
    }
    }

    // Speed limits
    v = std::clamp(v, -max_lin_, max_lin_);
    w = std::clamp(w, -max_ang_, max_ang_);

    // Acceleration AND braking limits. Braking is also limited: an instant stop from
    // high speed (e.g. TRANSLATE -> ROTATE at 0.8 m/s) tips the robot over.
    auto slew = [](double target, double last, double up, double down) {
        bool speeding_up = std::fabs(target) > std::fabs(last) && target * last >= 0.0;
        double step = speeding_up ? up : down;
        return std::clamp(target, last - step, last + step);
    };
    v = slew(v, last_lin_cmd_, max_lin_accel_ * dt_, max_lin_decel_ * dt_);
    w = slew(w, last_ang_cmd_, max_ang_accel_ * dt_, max_ang_decel_ * dt_);
    last_lin_cmd_ = v;
    last_ang_cmd_ = w;

    cmd.linear.x  = v;
    cmd.angular.z = w;
    return cmd;
}

void RTRController::resetPID()
{
    lin_error_sum_ = ang_error_sum_ = 0.0;
    lin_error_last_ = ang_error_last_ = std::nan("");
}

void RTRController::loadNextGoal()
{
    if (!goal_queue_.empty()) {
        goal_x_     = goal_queue_.front().x;
        goal_y_     = goal_queue_.front().y;
        goal_theta_ = goal_queue_.front().theta;
        goal_queue_.erase(goal_queue_.begin());
        RCLCPP_INFO(rclcpp::get_logger("rtr_controller"), "Moving to next goal at %.2f %.2f", goal_x_, goal_y_);
    } else {
        goal_received_ = false;
        RCLCPP_INFO(rclcpp::get_logger("rtr_controller"), "Plan complete");
    }
}
