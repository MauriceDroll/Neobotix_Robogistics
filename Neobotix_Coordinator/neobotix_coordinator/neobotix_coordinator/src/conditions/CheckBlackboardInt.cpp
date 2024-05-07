#include <neobotix_coordinator/conditions/CheckBlackboardInt.h>

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
BT::PortsList CheckBlackboardInt::providedPorts()
{
    return {BT::InputPort<int>("input"),
            BT::InputPort<int>("compare_to")};
}

/**
 * @brief Define what happens when this conditions is checked.
 * @return BT::NodeStatus SUCCESS or FAILURE
 */
BT::NodeStatus CheckBlackboardInt::on_check()
{
    if (ports.get_value<int>("input") == ports.get_value<int>("compare_to"))
    {
        log("values matched");
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::FAILURE;
}