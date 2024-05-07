#include <neobotix_coordinator/actions/MoveBase.h>

/**
 * @brief Set the name of the ROS2 action server to connect with.
 * @return Topic name as a string.
 */
std::string MoveBase::ros2_action_name()
{
    return "navigate_to_pose";
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
BT::PortsList MoveBase::providedPorts()
{
    return {BT::InputPort<std::string>("frame_id"),
            BT::InputPort<float>("x"),
            BT::InputPort<float>("y"),
            BT::InputPort<float>("quaternion_x"),
            BT::InputPort<float>("quaternion_y"),
            BT::InputPort<float>("quaternion_z"),
            BT::InputPort<float>("quaternion_w")};
}

/**
 * @brief Set the content of the goal message which is sent to the ROS2 action server.
 */
void MoveBase::on_send(NavigateToPoseAction::Goal &goal)
{
    if (ports.has_value<float>("quaternion_x") && ports.has_value<float>("quaternion_y") && ports.has_value<float>("quaternion_z") && ports.has_value<float>("quaternion_w"))
    {
        goal.pose.pose.orientation.x = ports.get_value<float>("quaternion_x");
        goal.pose.pose.orientation.y = ports.get_value<float>("quaternion_y");
        goal.pose.pose.orientation.z = ports.get_value<float>("quaternion_z");
        goal.pose.pose.orientation.w = ports.get_value<float>("quaternion_w");
    }
    else
    {
        goal.pose.pose.orientation.x = 0;
        goal.pose.pose.orientation.y = 0;
        goal.pose.pose.orientation.z = 0;
        goal.pose.pose.orientation.w = 1;
        log("Note: No quaternion values set. Using default values.");
    }

    if (ports.has_value<std::string>("frame_id"))
    {
        goal.pose.header.frame_id = ports.get_value<std::string>("frame_id");
    }
    else
    {
        goal.pose.header.frame_id = "map";
    }

    goal.pose.pose.position.x = ports.get_value<float>("x");
    goal.pose.pose.position.y = ports.get_value<float>("y");

    goal.pose.pose.position.z = 0; // z-value not neccessary
    goal.pose.header.stamp = get_node_handle()->now();

    log("Goal: (x=" + Converter::ftos(goal.pose.pose.position.x) + ", y=" + Converter::ftos(goal.pose.pose.position.y) + ")");
}

/**
 * @brief Define what happens when recieving feedback from the ROS2 action server.
 */
void MoveBase::on_feedback(const std::shared_ptr<const NavigateToPoseAction::Feedback>)
{
    // log("Current position: (x=" + Converter::ftos((float)feedback->current_pose.pose.position.x) +
    //     ", y=" + Converter::ftos((float)feedback->current_pose.pose.position.y) +
    //     "), Time elapsed: " + std::to_string(feedback->navigation_time.sec) + "s");
}

/**
 * @brief Define what happens when recieving the result from the ROS2 action server.
 */
void MoveBase::on_result(const rclcpp_action::ClientGoalHandle<NavigateToPoseAction>::WrappedResult &, const NavigateToPoseAction::Goal &goal)
{
    log("Goal reached! (x=" + Converter::ftos(goal.pose.pose.position.x) +
        ", y=" + Converter::ftos(goal.pose.pose.position.y) +
        "), Total time: " + std::to_string((int)get_node_handle()->now().seconds() - goal.pose.header.stamp.sec) + "s");
}