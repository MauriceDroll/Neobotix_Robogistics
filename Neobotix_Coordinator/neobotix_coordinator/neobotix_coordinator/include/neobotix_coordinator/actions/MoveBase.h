/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Action client for "navigate_to_pose"(NavigateToPose)
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2022.05.12)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosAction.h>
#include <nav2_msgs/action/navigate_to_pose.hpp>

using NavigateToPoseAction = nav2_msgs::action::NavigateToPose;

class MoveBase : public RosAction<NavigateToPoseAction>
{
public:
    static BT::PortsList providedPorts();

    MoveBase(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros2_action_name() override;

    void on_send(NavigateToPoseAction::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const NavigateToPoseAction::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<NavigateToPoseAction>::WrappedResult &result, const NavigateToPoseAction::Goal &goal) override;
};