#include <neobotix_coordinator/conditions/CheckBattery.h>

/**
 * @brief Constructor of the node, initialize e.g. ROS2 subscriber.
 */
CheckBattery::CheckBattery(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    battery_state_subscription_ = get_node_handle()->create_subscription<sensor_msgs::msg::BatteryState>("BatteryState", 10, [&](const sensor_msgs::msg::BatteryState::SharedPtr msg)
                                                                                                         { current_battery_level_ = msg->percentage; });
}

/**
 * @brief Set the list of ports provided by the BT node.
 *
 * New port:
 *      direction = [BT::InputPort, BT::OutputPort, BT::BidirectionalPort]
 *      data_type = <[float, int, std::string]>
 *      name = ("name")
 *
 * @return List of provided ports.
 */
BT::PortsList CheckBattery::providedPorts()
{
    return {BT::InputPort<float>("battery_level")};
}

/**
 * @brief Define what happens when this conditions is checked.
 * @return BT::NodeStatus SUCCESS or FAILURE
 */
BT::NodeStatus CheckBattery::on_check()
{
    if (current_battery_level_ <= ports.get_value<float>("battery_level"))
    {
        log("[BatteryState] Battery less than " + Converter::ftos(ports.get_value<float>("battery_level") * 100) + "%", LogLevel::Warn);

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}