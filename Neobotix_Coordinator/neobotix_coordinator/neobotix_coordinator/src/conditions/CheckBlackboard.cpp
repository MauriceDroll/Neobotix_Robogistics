#include <neobotix_coordinator/conditions/CheckBlackboard.h>

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
BT::PortsList CheckBlackboard::providedPorts()
{
    return {BT::InputPort<std::string>("input"),
            BT::InputPort<std::string>("compare_to")};
}

/**
 * @brief Define what happens when this conditions is checked.
 * @return BT::NodeStatus SUCCESS or FAILURE
 */
BT::NodeStatus CheckBlackboard::on_check()
{
    if (ports.get_value<std::string>("input") == ports.get_value<std::string>("compare_to"))
    {
        log("values matched");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}