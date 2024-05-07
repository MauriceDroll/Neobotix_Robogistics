#include <neobotix_coordinator/services/ClearLocalCostmap.h>

/**
 * @brief Set the name of the ROS2 service server to connect with.
 * @return Topic name as a string.
 */
std::string ClearLocalCostmap::ros2_service_name()
{
    return "/local_costmap/clear_entirely_local_costmap";
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
BT::PortsList ClearLocalCostmap::providedPorts()
{
    return {};
}

/**
 * @brief Set the content of the request message which is sent to the ROS2 service server.
 */
void ClearLocalCostmap::on_send(std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
}

/**
 * @brief Define what happens when recieving the response from the ROS2 service server.
 */
bool ClearLocalCostmap::on_result(std::shared_ptr<ClearLocalCostmapSrv::Response>, std::shared_ptr<ClearLocalCostmapSrv::Request>)
{
    log("Local costmap cleared");
    return true;
}