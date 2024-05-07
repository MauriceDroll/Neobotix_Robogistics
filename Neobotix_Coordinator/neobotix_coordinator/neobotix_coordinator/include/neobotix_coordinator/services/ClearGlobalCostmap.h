/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Provides the ROS2-Service client "ClearGlobalCostmapSrv"
 *
 * @author Andreas Zachariae
 * @since 1.0.0 (2023.05.19)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosService.h>
#include <nav2_msgs/srv/clear_entire_costmap.hpp>

using ClearGlobalCostmapSrv = nav2_msgs::srv::ClearEntireCostmap;

class ClearGlobalCostmap : public RosService<ClearGlobalCostmapSrv>
{
public:
    static BT::PortsList providedPorts();

    ClearGlobalCostmap(const std::string &name, const BT::NodeConfiguration &config) : RosService(name, config) {}

    std::string ros2_service_name() override;

    void on_send(std::shared_ptr<ClearGlobalCostmapSrv::Request> request) override;
    bool on_result(std::shared_ptr<ClearGlobalCostmapSrv::Response> response, std::shared_ptr<ClearGlobalCostmapSrv::Request> request) override;
};