/**
 * @file classical_frontier_detection.cpp
 * @brief Implementation of frontier-based exploration
 */

#include "frontier_exploration/classical_frontier_detector.hpp"

#include <limits>
#include <std_msgs/msg/bool.hpp>

namespace
{

rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr
    exploration_complete_publisher_;

void set_invalid_frontier_response(
    const rclcpp::Clock::SharedPtr & clock,
    const std::shared_ptr<
        frontier_interfaces::srv::FrontierGoal::Response
    > & response
)
{
    geometry_msgs::msg::PoseStamped invalid_goal;
    invalid_goal.header.stamp = clock->now();
    invalid_goal.header.frame_id = "";
    invalid_goal.pose.position.x =
        std::numeric_limits<double>::quiet_NaN();
    invalid_goal.pose.position.y =
        std::numeric_limits<double>::quiet_NaN();
    invalid_goal.pose.position.z = 0.0;
    invalid_goal.pose.orientation.x = 0.0;
    invalid_goal.pose.orientation.y = 0.0;
    invalid_goal.pose.orientation.z = 0.0;
    invalid_goal.pose.orientation.w = 1.0;
    response->goal_pose = invalid_goal;
}

}  // namespace

FrontierExplorer::FrontierExplorer()
: Node("frontier_explorer")
{
    region_size_thresh_ = this->declare_parameter(
        "region_size_thresh",
        12
    );

    robot_width_ = this->declare_parameter(
        "robot_width",
        0.5
    );

    occupancy_map_topic_ = this->declare_parameter(
        "occupancy_map_msg",
        "map"
    );

    map_subscription_ =
        this->create_subscription<nav_msgs::msg::OccupancyGrid>(
            occupancy_map_topic_,
            1,
            std::bind(
                &FrontierExplorer::map_callback,
                this,
                _1
            )
        );

    service_ =
        this->create_service<frontier_interfaces::srv::FrontierGoal>(
            "frontier_pose",
            std::bind(
                &FrontierExplorer::get_frontiers,
                this,
                _1,
                _2
            )
        );

    marker_publisher_ =
        this->create_publisher<visualization_msgs::msg::Marker>(
            "f_markers",
            1
        );

    frontier_map_publisher_ =
        this->create_publisher<nav_msgs::msg::OccupancyGrid>(
            "f_map",
            1
        );

    // Retain the latest completion state for late-starting subscribers.
    rclcpp::QoS completion_qos(rclcpp::KeepLast(1));
    completion_qos.reliable();
    completion_qos.transient_local();

    exploration_complete_publisher_ =
        this->create_publisher<std_msgs::msg::Bool>(
            "/frontier_exploration/mapping_complete",
            completion_qos
        );


    tf_buffer_ =
        std::make_unique<tf2_ros::Buffer>(
            this->get_clock()
        );

    tf_listener_ =
        std::make_shared<tf2_ros::TransformListener>(
            *tf_buffer_
        );
}

void FrontierExplorer::map_callback(
    const nav_msgs::msg::OccupancyGrid::SharedPtr recent_map
)
{
    std::lock_guard<std::mutex> guard(mutex_);
    map_ = *recent_map;
}

void FrontierExplorer::get_frontiers(
    const std::shared_ptr<
        frontier_interfaces::srv::FrontierGoal::Request
    > request,
    std::shared_ptr<
        frontier_interfaces::srv::FrontierGoal::Response
    > response
)
{
    RCLCPP_INFO(
        this->get_logger(),
        "Received request, %d",
        request->goal_rank
    );

    std::unique_lock<std::mutex> lck(mutex_);
    nav_msgs::msg::OccupancyGrid map = map_;
    lck.unlock();

    if (
        map.data.empty() ||
        map.info.width == 0 ||
        map.info.height == 0
    ) {
        RCLCPP_WARN(
            this->get_logger(),
            "No valid map received yet."
        );
        set_invalid_frontier_response(this->get_clock(), response);
        return;
    }

    std::vector<cell> processed = preprocessMap(
        map.data,
        map.info.width,
        map.info.height,
        5
    );

    frontierCellGrid_.clear();
    frontierCellGrid_ = computeFrontierCellGrid(
        processed,
        map.info.width,
        map.info.height
    );

    frontierRegions_.clear();
    frontierRegions_ = computeFrontierRegions(
        frontierCellGrid_,
        map.info.width,
        map.info.height,
        map.info.resolution,
        map.info.origin.position.x,
        map.info.origin.position.y,
        region_size_thresh_
    );

    std_msgs::msg::Bool exploration_complete_message;
    exploration_complete_message.data = frontierRegions_.empty();
    exploration_complete_publisher_->publish(
        exploration_complete_message
    );

    RCLCPP_INFO(
        this->get_logger(),
        "Exploration complete: %s. Frontier count: %zu.",
        exploration_complete_message.data ? "true" : "false",
        frontierRegions_.size()
    );

    if (
        request->goal_rank < 0 ||
        static_cast<std::size_t>(request->goal_rank) >=
            frontierRegions_.size()
    ) {
        RCLCPP_WARN(
            this->get_logger(),
            "Requested frontier rank %d, but only %zu frontiers exist.",
            request->goal_rank,
            frontierRegions_.size()
        );
        set_invalid_frontier_response(this->get_clock(), response);
        return;
    }

    nav_msgs::msg::OccupancyGrid f_map = map;
    f_map.data = processed;
    frontier_map_publisher_->publish(f_map);
    publishFrontiers();

    geometry_msgs::msg::TransformStamped stransform;

    try {
        stransform = tf_buffer_->lookupTransform(
            odom_frame_,
            base_frame_,
            tf2::TimePointZero,
            tf2::durationFromSec(3)
        );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_ERROR(
            this->get_logger(),
            "%s",
            ex.what()
        );
        set_invalid_frontier_response(this->get_clock(), response);
        return;
    }

    frontierRegion goal = selectFrontier(
        frontierRegions_,
        request->goal_rank,
        stransform.transform.translation.x,
        stransform.transform.translation.y
    );

    geometry_msgs::msg::PoseStamped goal_pose;
    goal_pose.header.stamp = this->get_clock()->now();
    goal_pose.header.frame_id = map_frame_;
    goal_pose.pose.position.x = goal.x;
    goal_pose.pose.position.y = goal.y;
    goal_pose.pose.position.z = 0.0;
    goal_pose.pose.orientation.x = 0.0;
    goal_pose.pose.orientation.y = 0.0;
    goal_pose.pose.orientation.z = 0.0;
    goal_pose.pose.orientation.w = 1.0;

    response->goal_pose = goal_pose;

    RCLCPP_INFO(
        this->get_logger(),
        "Sending goal x: %f y: %f.",
        goal_pose.pose.position.x,
        goal_pose.pose.position.y
    );
}

void FrontierExplorer::publishFrontiers()
{
    visualization_msgs::msg::Marker::SharedPtr sphere_list(
        new visualization_msgs::msg::Marker
    );

    sphere_list->header.frame_id = map_frame_;
    sphere_list->header.stamp = this->get_clock()->now();
    sphere_list->type = visualization_msgs::msg::Marker::SPHERE_LIST;
    sphere_list->action = visualization_msgs::msg::Marker::ADD;
    sphere_list->scale.x = 0.1;
    sphere_list->scale.y = 0.1;
    sphere_list->scale.z = 0.1;
    sphere_list->color.g = 1.0;
    sphere_list->color.a = 1.0;

    for (const auto & reg : frontierRegions_) {
        geometry_msgs::msg::Point p;
        p.x = reg.x;
        p.y = reg.y;
        p.z = 0.05;
        sphere_list->points.push_back(p);
    }

    marker_publisher_->publish(*sphere_list);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<FrontierExplorer>());
    rclcpp::shutdown();
    return 0;
}
