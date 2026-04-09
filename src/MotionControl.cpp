#include "mpc_rbt_simulator/RobotConfig.hpp"
#include "MotionControl.hpp"


MotionControlNode::MotionControlNode() :
    rclcpp::Node("motion_control_node") {

	// Subscribers
	odometry_sub_ = create_subscription<nav_msgs::msg::Odometry>("/odometry", 10, std::bind(&MotionControlNode::odomCallback, this, std::placeholders::_1));

	lidar_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("/tiago_base/Hokuyo_URG_04LX_UG01", 10, std::bind(&MotionControlNode::lidarCallback, this, std::placeholders::_1));

	// Publisher
	twist_publisher_ = create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

	// Client
	plan_client_ = create_client<nav_msgs::srv::GetPlan>("/plan_path");

	// Action server
	nav_server_ = rclcpp_action::create_server<nav2_msgs::action::NavigateToPose>(
		this,
	        "/go_to_goal",
		std::bind(&MotionControlNode::navHandleGoal, this, std::placeholders::_1, std::placeholders::_2),
		std::bind(&MotionControlNode::navHandleCancel, this, std::placeholders::_1),
		std::bind(&MotionControlNode::navHandleAccepted, this, std::placeholders::_1));

	// Inicializácia stavov
	navigating_ = false;
	collision_detected_ = false;
	current_waypoint_ = 0;
	
	lookahead_dist_ = 0.5;
	max_linear_vel_ = 0.2;
	max_angular_vel_ = 1.0;
	collision_threshold_ = 0.5;

        RCLCPP_INFO(get_logger(), "Motion control node started.");

       
    }

void MotionControlNode::checkCollision() {

    if (laser_scan_.ranges.empty()) {
        return;
    }

    int center = laser_scan_.ranges.size() / 2;
    int width = 30;

    for (int i = center - width; i < center + width; i++) {
	RCLCPP_INFO(get_logger(), "Lidar fine.");
        if (i < 0 || i >= (int)laser_scan_.ranges.size())
            continue;

        if (laser_scan_.ranges[i] < collision_threshold_) {
	RCLCPP_INFO(get_logger(), "Lidar collision detected.");
            geometry_msgs::msg::Twist stop;
            twist_publisher_->publish(stop);

            collision_detected_ = true;
            navigating_ = false;

            return;
        }
    }
}

void MotionControlNode::updateTwist() {

    if (!navigating_ || path_.poses.empty())
        return;

    if (current_waypoint_ >= path_.poses.size())
        return;

    auto target = path_.poses[current_waypoint_];
    auto current = current_pose_;

    double dx = target.pose.position.x - current.pose.position.x;
    double dy = target.pose.position.y - current.pose.position.y;

    tf2::Quaternion q(
        current.pose.orientation.x,
        current.pose.orientation.y,
        current.pose.orientation.z,
        current.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Uhol k cieľu
    double target_angle = atan2(dy, dx);
    double angle_error = target_angle - yaw;

    // normalizácia do <-pi, pi>
    while (angle_error > M_PI) angle_error -= 2*M_PI;
    while (angle_error < -M_PI) angle_error += 2*M_PI;

    geometry_msgs::msg::Twist twist;

    if (fabs(angle_error) > 0.5) {
        // najprv sa otoč
        twist.linear.x = 0.0;
        twist.angular.z = 0.8 * angle_error;
    } else {
        // potom choď dopredu
        twist.linear.x = max_linear_vel_;
        twist.angular.z = 1.5 * angle_error;
    }

    // saturácia
    if (twist.angular.z > max_angular_vel_) twist.angular.z = max_angular_vel_;
    if (twist.angular.z < -max_angular_vel_) twist.angular.z = -max_angular_vel_;

    twist_publisher_->publish(twist);

    // prepnutie waypointu
    double dist = sqrt(dx * dx + dy * dy);
    if (dist < 0.2) {
        current_waypoint_++;
    }
}

rclcpp_action::GoalResponse MotionControlNode::navHandleGoal(const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const nav2_msgs::action::NavigateToPose::Goal> goal)
{
    (void)uuid;

    goal_pose_ = goal->pose;

    RCLCPP_INFO(get_logger(), "Goal received");

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionControlNode::navHandleCancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    (void)goal_handle;

    navigating_ = false;

    return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionControlNode::navHandleAccepted(
    const std::shared_ptr<rclcpp_action::ServerGoalHandle<nav2_msgs::action::NavigateToPose>> goal_handle)
{
    goal_handle_ = goal_handle;

    auto request = std::make_shared<nav_msgs::srv::GetPlan::Request>();
    request->start = current_pose_;
    request->goal = goal_pose_;

    RCLCPP_INFO(get_logger(), "Goal accpeted");
	
    plan_client_->async_send_request(request, std::bind(&MotionControlNode::pathCallback, this, std::placeholders::_1)
    );
}

void MotionControlNode::execute() {

    if (!goal_handle_) {
    return;
    }

    RCLCPP_INFO(get_logger(), "Executing...");

    rclcpp::Rate loop_rate(1.0);

    while (rclcpp::ok() && navigating_) {

        if (collision_detected_) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->abort(result);
            return;
        }

        if (current_waypoint_ >= path_.poses.size()) {
            auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
            goal_handle_->succeed(result);
            navigating_ = false;
            return;
        }

        loop_rate.sleep();
    }
}

void MotionControlNode::pathCallback(rclcpp::Client<nav_msgs::srv::GetPlan>::SharedFuture future)
{
    auto response = future.get();

    if (response->plan.poses.empty()) {
        RCLCPP_ERROR(get_logger(), "No path found!");
        auto result = std::make_shared<nav2_msgs::action::NavigateToPose::Result>();
    	goal_handle_->abort(result);
        return;
    }

    path_ = response->plan;
    current_waypoint_ = 0;
    navigating_ = true;
    collision_detected_ = false;

    std::thread(&MotionControlNode::execute, this).detach();
}

void MotionControlNode::odomCallback(const nav_msgs::msg::Odometry & msg) {
    current_pose_.pose = msg.pose.pose;

    if (navigating_) {
        checkCollision();
        updateTwist();
    }
}

void MotionControlNode::lidarCallback(const sensor_msgs::msg::LaserScan & msg) {
    laser_scan_ = msg;
}
