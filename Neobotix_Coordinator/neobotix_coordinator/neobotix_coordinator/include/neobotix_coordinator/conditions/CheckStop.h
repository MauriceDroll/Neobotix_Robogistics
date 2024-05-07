/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Checks the ROS2-Topic "Stop"
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <std_msgs/msg/empty.hpp>

#include <iras_behaviortree_ros2/components/RosCondition.h>

class CheckStop : public RosCondition
{
public:
    static BT::PortsList providedPorts();

    CheckStop(const std::string &name, const BT::NodeConfiguration &config);

    BT::NodeStatus on_check() override;

private:
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr stop_subscription_;

    bool stop_recieved_ = false;
};
