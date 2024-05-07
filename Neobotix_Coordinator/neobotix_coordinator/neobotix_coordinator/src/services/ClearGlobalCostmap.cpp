#include <neobotix_coordinator/services/ClearGlobalCostmap.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string ClearGlobalCostmap::ros2_service_name()
{
    return "/global_costmap/clear_entirely_global_costmap";
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
BT::PortsList ClearGlobalCostmap::providedPorts()
{
    return {};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void ClearGlobalCostmap::on_send(std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool ClearGlobalCostmap::on_result(std::shared_ptr<ClearGlobalCostmapSrv::Response>, std::shared_ptr<ClearGlobalCostmapSrv::Request>)
{
    log("Global costmap cleared");
    return true;
}