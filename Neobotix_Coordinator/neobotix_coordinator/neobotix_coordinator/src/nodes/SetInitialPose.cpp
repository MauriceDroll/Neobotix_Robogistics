#include <neobotix_coordinator/nodes/SetInitialPose.h>

/**
 * @brief Constructor of the node, initialize e.g. ROS2 subscriber.
 */
SetInitialPose::SetInitialPose(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    pose_publisher_ = get_node_handle()->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>("/initialpose", 10);
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
BT::PortsList SetInitialPose::providedPorts()
{
    return {BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z"),
            BT::InputPort<float>("quaternion_x"),
            BT::InputPort<float>("quaternion_y"),
            BT::InputPort<float>("quaternion_z"),
            BT::InputPort<float>("quaternion_w")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus SetInitialPose::on_start()
{
    geometry_msgs::msg::PoseWithCovarianceStamped msg;

    msg.pose.pose.position.x = ports.get_value<float>("x");
    msg.pose.pose.position.y = ports.get_value<float>("y");
    msg.pose.pose.position.z = ports.get_value<float>("z");
    msg.pose.pose.orientation.x = ports.get_value<float>("quaternion_x");
    msg.pose.pose.orientation.y = ports.get_value<float>("quaternion_y");
    msg.pose.pose.orientation.z = ports.get_value<float>("quaternion_z");
    msg.pose.pose.orientation.w = ports.get_value<float>("quaternion_w");

    pose_publisher_->publish(msg);

    log("Goal: (x=" + Converter::ftos(msg.pose.pose.position.x) + ", y=" + Converter::ftos(msg.pose.pose.position.y) + ", z=" + Converter::ftos(msg.pose.pose.position.z) + ")");

    return BT::NodeStatus::SUCCESS;
}