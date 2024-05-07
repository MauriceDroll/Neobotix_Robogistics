/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Checks the ROS2-Topic "DiagnosticStatus"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <std_msgs/msg/empty.hpp>
#include <diagnostic_msgs/msg/diagnostic_status.hpp>

#include <iras_behaviortree_ros2/components/RosCondition.h>

class CheckDiagnosticStatus : public RosCondition
{
public:
    static BT::PortsList providedPorts();

    CheckDiagnosticStatus(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Publisher<std_msgs::msg::Empty>::SharedPtr stop_publisher_;
    rclcpp::Subscription<diagnostic_msgs::msg::DiagnosticStatus>::SharedPtr diagnostic_status_subscription_;

    bool diagnostic_error_ = false;
};
