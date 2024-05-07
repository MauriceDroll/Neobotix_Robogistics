/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "CloseGripperSrv"
 *
 * @author Maurice Droll, Andreas Schmitt, Leo Schäfer 
 * @since 1.0.0 (2024.03.05)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <iras_interfaces/srv/move_gripper.hpp> 

using CloseGripperSrv = iras_interfaces::srv::MoveGripper;

class CloseGripper : public RosService<CloseGripperSrv>
{
public:
    static BT::PortsList providedPorts();

    CloseGripper(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<CloseGripperSrv::Request> request) override;
    bool on_result(std::shared_ptr<CloseGripperSrv::Response> response, std::shared_ptr<CloseGripperSrv::Request> request) override;
};