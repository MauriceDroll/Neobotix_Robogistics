#include <neobotix_coordinator/conditions/CheckStop.h>

/**
 * @brief Constructor of the node, initialize e.g. ROS2 subscriber.
 */
CheckStop::CheckStop(const std::string &name, const BT::NodeConfiguration &config) : RosCondition(name, config)
{
    stop_subscription_ = get_node_handle()->create_subscription<std_msgs::msg::Empty>("Stop", 10, [&](const std_msgs::msg::Empty::SharedPtr)
                                                                                      { stop_recieved_ = true; });
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
BT::PortsList CheckStop::providedPorts()
{
    return {};
}

/**
 * @brief Define what happens when this conditions is checked.
 * @return BT::NodeStatus SUCCESS or FAILURE
 */
BT::NodeStatus CheckStop::on_check()
{
    if (stop_recieved_)
    {
        log("Stop recieved");

        return BT::NodeStatus::FAILURE;
    }

    return BT::NodeStatus::SUCCESS;
}