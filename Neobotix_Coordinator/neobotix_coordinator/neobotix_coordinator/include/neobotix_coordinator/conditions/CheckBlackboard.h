/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "IRASCoordinator"
 * Purpose : Checks if the given value macthes the blackboard entry
 *
 * @author Andreas Zachariae
 * @author Moritz Weisenböhler
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#pragma once

#include <neobotix_coordinator/default.h>

#include <iras_behaviortree_ros2/components/RosCondition.h>

class CheckBlackboard : public RosCondition
{
public:
    static BT::PortsList providedPorts();

    CheckBlackboard(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config) {}

    BT::NodeStatus on_check() override;
};
