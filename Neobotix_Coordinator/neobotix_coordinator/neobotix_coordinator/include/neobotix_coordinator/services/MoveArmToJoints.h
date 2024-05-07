/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "MoveToJointPositionSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <iras_interfaces/srv/move_to_joint_position.hpp>

using MoveToJointPositionSrv = iras_interfaces::srv::MoveToJointPosition;

class MoveArmToJoints : public RosService<MoveToJointPositionSrv>
{
public:
    static BT::PortsList providedPorts();

    MoveArmToJoints(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<MoveToJointPositionSrv::Request> request) override;
    bool on_result(std::shared_ptr<MoveToJointPositionSrv::Response> response, std::shared_ptr<MoveToJointPositionSrv::Request> request) override;
};