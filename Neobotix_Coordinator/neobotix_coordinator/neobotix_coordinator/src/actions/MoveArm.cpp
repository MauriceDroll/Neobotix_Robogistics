#include <neobotix_coordinator/actions/MoveArm.h>

#define DEG2RAD(x) ((x) * 3.1415 / 180.0f)

/**
 * @brief Set the name of the ROS2 action server to connect with.
 * @return Topic name as a string.
 */
std::string MoveArm::ros2_action_name()
{
    return "move_to_pose";
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
BT::PortsList MoveArm::providedPorts()
{
    return {BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("z"),
            BT::InputPort<float>("rotation_x"),
            BT::InputPort<float>("rotation_y"),
            BT::InputPort<float>("rotation_z"),
            BT::InputPort<bool>("cartesian"),
            BT::InputPort<float>("speed")};
}

/**
 * @brief Set the content of the goal message which is sent to the ROS2 action server.
 */
void MoveArm::on_send(MoveArmMoveIt::Goal &goal)
{
    goal.pose.pose.position.x = ports.get_value<float>("x");
    goal.pose.pose.position.y = ports.get_value<float>("y");
    goal.pose.pose.position.z = ports.get_value<float>("z");

    tf2::Quaternion q;
    q.setRPY(DEG2RAD(ports.get_value<float>("rotation_x")), DEG2RAD(ports.get_value<float>("rotation_y")), DEG2RAD(ports.get_value<float>("rotation_z")));
    q.normalize();
    goal.pose.pose.orientation = tf2::toMsg(q);

    goal.cart = ports.get_value<bool>("cartesian");
    goal.speed = ports.get_value<float>("speed");

    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        ", z=" + Converter::ftos(goal.pose.pose.position.z) +
        ")\n(rx=" + Converter::ftos(ports.get_value<float>("rotation_x")) +
        ", ry=" + Converter::ftos(ports.get_value<float>("rotation_y")) +
        ", rz=" + Converter::ftos(ports.get_value<float>("rotation_z")) + ")");
}

/**
 * @brief Define what happens when recieving feedback from the ROS2 action server.
 */
void MoveArm::on_feedback(const std::shared_ptr<const MoveArmMoveIt::Feedback> feedback)
{
    log("Status: " + feedback->status_code);
}

/**
 * @brief Define what happens when recieving the result from the ROS2 action server.
 */
void MoveArm::on_result(const rclcpp_action::ClientGoalHandle<MoveArmMoveIt>::WrappedResult &result, const MoveArmMoveIt::Goal &goal)
{
    log("Goal reached! (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        ", z=" + Converter::ftos(goal.pose.pose.position.z) +
        "), Total time: " + std::to_string((int)(get_node_handle()->now().seconds() - -goal.pose.header.stamp.sec)) + "s" +
        ", code: " + std::to_string(result.result->err_code) + " " + result.result->msg);
}