/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Wait in seconds
 *
 * @author Andreas Zachariae
 * @since 1.1.0 (2021.09.06)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosNode.h>

#include <time.h>

class Wait : public RosNode
{
public:
    static BT::PortsList providedPorts();

    Wait(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config) {}

    BT::NodeStatus on_start() override;
    BT::NodeStatus on_running() override;
    void on_halted() override;

private:
    time_t start_time_;
};