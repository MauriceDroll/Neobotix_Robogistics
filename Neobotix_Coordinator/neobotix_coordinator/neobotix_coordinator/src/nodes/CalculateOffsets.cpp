#include <neobotix_coordinator/nodes/CalculateOffsets.h>

/**
 * @brief Constructor of the node, initialize e.g. ROS2 subscriber.
 */
CalculateOffsets::CalculateOffsets(const std::string &name, const BT::NodeConfiguration &config) : RosNode(name, config)
{
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_node_handle()->get_clock());
    transform_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
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
BT::PortsList CalculateOffsets::providedPorts()
{
    return {BT::InputPort<float>("offset_x"),
            BT::InputPort<float>("offset_y"),
            BT::InputPort<float>("offset_z"),
            BT::OutputPort<float>("out_x"),
            BT::OutputPort<float>("out_y"),
            BT::OutputPort<float>("out_z"),
            BT::OutputPort<float>("out_qx"),
            BT::OutputPort<float>("out_qy"),
            BT::OutputPort<float>("out_qz"),
            BT::OutputPort<float>("out_qw"),
            BT::InputPort<std::string>("base_frame"),
            BT::InputPort<std::string>("marker_frame"),
            BT::InputPort<std::string>("offset_frame"),
            BT::InputPort<int>("max_seconds")};
}

/**
 * @brief Define what happens when this node is ticked for the first time.
 * @return BT::NodeStatus RUNNING (Has to return RUNNING to allow on_running to be called)
 */
BT::NodeStatus CalculateOffsets::on_start()
{
    start_time_ = time(NULL);
    last_checked_ = time(NULL);

    tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(get_node_handle());
    rclcpp::Time now = get_node_handle()->get_clock()->now();
    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = now;
    t.header.frame_id = ports.get_value<std::string>("marker_frame");
    t.child_frame_id = ports.get_value<std::string>("offset_frame");

    t.transform.translation.x = ports.get_value<float>("offset_x");
    t.transform.translation.y = ports.get_value<float>("offset_y");
    t.transform.translation.z = ports.get_value<float>("offset_z");
    tf2::Quaternion q;
    // Could be extended to also specify rotation offset
    q.setRPY(
        0.0,
        0.0,
        0.0);
    t.transform.rotation.x = q.x();
    t.transform.rotation.y = q.y();
    t.transform.rotation.z = q.z();
    t.transform.rotation.w = q.w();

    tf_publisher_->sendTransform(t);

    return BT::NodeStatus::RUNNING;
}

/**
 * @brief Define what happens when this node is ticked in RUNNING mode.
 * @return BT::NodeStatus SUCCESS or FAILURE or RUNNING
 */
BT::NodeStatus CalculateOffsets::on_running()
{
    if ((time(NULL) - start_time_) < ports.get_value<int>("max_seconds"))
    {
        if ((time(NULL) - last_checked_) > 1)
        {
            geometry_msgs::msg::TransformStamped transformStamped;
            last_checked_ = time(NULL);

            try
            {
                transformStamped = tf_buffer_->lookupTransform(
                    ports.get_value<std::string>("base_frame"), ports.get_value<std::string>("offset_frame"),
                    tf2::TimePointZero);
            }
            catch (tf2::TransformException &ex)
            {
                RCLCPP_INFO(
                    get_node_handle()->get_logger(), "Could not transform: %s", ex.what());

                return BT::NodeStatus::RUNNING;
            }

            ports.set_value<float>("out_x", transformStamped.transform.translation.x);
            ports.set_value<float>("out_y", transformStamped.transform.translation.y);
            ports.set_value<float>("out_z", transformStamped.transform.translation.z);
            ports.set_value<float>("out_qx", transformStamped.transform.rotation.x);
            ports.set_value<float>("out_qy", transformStamped.transform.rotation.y);
            ports.set_value<float>("out_qz", transformStamped.transform.rotation.z);
            ports.set_value<float>("out_qw", transformStamped.transform.rotation.w);

            log("Offset Position in base_frame: (x=" + Converter::ftos(transformStamped.transform.translation.x) + ", y=" + Converter::ftos(transformStamped.transform.translation.y) + ", z=" + Converter::ftos(transformStamped.transform.translation.z) + ")");

            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            return BT::NodeStatus::RUNNING;
        }
    }
    return BT::NodeStatus::FAILURE;
}
