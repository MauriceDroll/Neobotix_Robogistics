/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Add offsets from "marker_frame" to new "offset_frame" defined in given "base_frame"
 *
 * @author Andreas Zachariae
 * @author Frederik Plahl
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>

#include <geometry_msgs/msg/transform_stamped.hpp>

#include <rclcpp/rclcpp.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <chrono>
#include <memory>
#include <time.h>

class CalculateOffsets : public RosNode
{
public:
    static BT::PortsList providedPorts();

    CalculateOffsets(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override {}

private:
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> transform_listener_{nullptr};
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
    time_t start_time_;
    time_t last_checked_;
};