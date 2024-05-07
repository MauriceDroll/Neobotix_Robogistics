/*********************************************************
 * Neobotix - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Action client for "move_to_pose"(MoveArmMoveIt)
 *
 * @author Philipp Kirsch
 * @since 1.0.0 (2022.06.28)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosAction.h>
#include <iras_interfaces/action/move_arm_move_it.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

using MoveArmMoveIt = iras_interfaces::action::MoveArmMoveIt;

class MoveArm : public RosAction<MoveArmMoveIt>
{
public:
    static BT::PortsList providedPorts();

    MoveArm(const std::string &name, const BT::NodeConfiguration &config) : RosAction(name, config) {}

    std::string ros2_action_name() override;

    void on_send(MoveArmMoveIt::Goal &goal) override;
    void on_feedback(const std::shared_ptr<const MoveArmMoveIt::Feedback> feedback) override;
    void on_result(const rclcpp_action::ClientGoalHandle<MoveArmMoveIt>::WrappedResult &result, const MoveArmMoveIt::Goal &goal) override;
};