#include <neobotix_coordinator/nodes/Wait.h>

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
BT::PortsList Wait::providedPorts()
{
    return {BT::InputPort<int>("seconds")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus Wait::on_start()
{
    start_time_ = time(NULL);

    log("Wait for " + std::to_string(ports.get_value<int>("seconds")) + " s...");

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is ticked in RUNNING mode.
 * @return BT::NodeStatus SUCCESS or FAILURE or RUNNING
 */
BT::NodeStatus Wait::on_running()
{
    if ((time(NULL) - start_time_) > ports.get_value<int>("seconds"))
    {
        return BT::NodeStatus::SUCCESS;
    }

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is halted.
 */
void Wait::on_halted()
{
}