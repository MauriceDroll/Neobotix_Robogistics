/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "ClearLocalCostmapSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using ClearLocalCostmapSrv = nav2_msgs::srv::ClearEntireCostmap;

class ClearLocalCostmap : public RosService<ClearLocalCostmapSrv>
{
public:
    static BT::PortsList providedPorts();

    ClearLocalCostmap(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<ClearLocalCostmapSrv::Request> request) override;
    bool on_result(std::shared_ptr<ClearLocalCostmapSrv::Response> response, std::shared_ptr<ClearLocalCostmapSrv::Request> request) override;
};