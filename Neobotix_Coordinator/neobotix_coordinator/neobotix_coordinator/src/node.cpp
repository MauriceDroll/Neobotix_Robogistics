/** *******************************************************
 * IRAS - University of Applied Sciences Karlsruhe
 * Module : ROS2-Node "Coordinator"
 * Purpose : main-function and start of the behavior tree
 *
 * @author Moritz Weisenböhler
 * @author Andreas Zachariae
 * @since 1.0.0 (2020.08.26)
 *********************************************************/
#include <behaviortree_cpp_v3/bt_factory.h>
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
#include <iras_behaviortree_ros2/components/RosInterface.h>
#include <iras_behaviortree_ros2/tools/XmlGenerator.h>

// Conditions
#include <neobotix_coordinator/conditions/CheckStop.h>
#include <neobotix_coordinator/conditions/CheckBattery.h>
#include <neobotix_coordinator/conditions/CheckDiagnosticStatus.h>
#include <neobotix_coordinator/conditions/CheckBlackboard.h>
#include <neobotix_coordinator/conditions/CheckBlackboardInt.h>

// User Input
#include <neobotix_coordinator/services/ParameterRequest.h>

// Arm
#include <neobotix_coordinator/actions/MoveArm.h>
#include <neobotix_coordinator/services/MoveArmToJoints.h>
#include <neobotix_coordinator/services/MoveArmToPose.h>

// Navigation
#include <neobotix_coordinator/actions/MoveBase.h>
#include <neobotix_coordinator/services/ClearGlobalCostmap.h>
#include <neobotix_coordinator/services/ClearLocalCostmap.h>
#include <neobotix_coordinator/nodes/SetInitialPose.h>

// Camera
#include <neobotix_coordinator/nodes/CalculateOffsets.h>
#include <neobotix_coordinator/nodes/GetMarkerPosition.h>
// #include <neobotix_coordinator/services/DetectObject.h>

// Gripper 
#include <neobotix_coordinator/services/OpenGripper.h>
#include <neobotix_coordinator/services/CloseGripper.h>


// Misc
#include <neobotix_coordinator/nodes/Wait.h>

BT::Tree create_tree(const std::string &main_tree_path, const std::string &groot_palette_path)
{
    BT::BehaviorTreeFactory factory;
    // Conditions
    factory.registerNodeType<CheckStop>("CheckStop");
    factory.registerNodeType<CheckBattery>("CheckBattery");
    factory.registerNodeType<CheckDiagnosticStatus>("CheckDiagnosticStatus");
    factory.registerNodeType<CheckBlackboard>("CheckBlackboard");
    factory.registerNodeType<CheckBlackboardInt>("CheckBlackboardInt");

    // User Input
    factory.registerNodeType<ParameterRequest<bool>>("BoolParameterRequest");
    factory.registerNodeType<ParameterRequest<int>>("IntParameterRequest");
    factory.registerNodeType<ParameterRequest<float>>("FloatParameterRequest");
    factory.registerNodeType<ParameterRequest<std::string>>("StringParameterRequest");

    // Arm
    factory.registerNodeType<MoveArm>("MoveArm");
    factory.registerNodeType<MoveArmToJoints>("MoveArmToJoints");
    factory.registerNodeType<MoveArmToPose>("MoveArmToPose");

    // Navigation
    factory.registerNodeType<MoveBase>("MoveBase");
    factory.registerNodeType<ClearGlobalCostmap>("ClearGlobalCostmap");
    factory.registerNodeType<ClearLocalCostmap>("ClearLocalCostmap");
    factory.registerNodeType<SetInitialPose>("SetInitialPose");

    // Camera
    factory.registerNodeType<CalculateOffsets>("CalculateOffsets");
    factory.registerNodeType<GetMarkerPosition>("GetMarkerPosition");
    //factory.registerNodeType<DetectObject>("DetectObject");

    // Misc
    factory.registerNodeType<Wait>("Wait");

    // Gripper
    factory.registerNodeType<OpenGripper>("OpenGripper");
    factory.registerNodeType<CloseGripper>("CloseGripper");


    XmlGenerator xml_generator(&factory);
    xml_generator.generate_xml_palette(groot_palette_path);

    return factory.createTreeFromFile(main_tree_path);
}

BT::NodeStatus run_tree(BT::Tree &tree)
{
    BT::NodeStatus status;
    BT::NodeStatus last_status = BT::NodeStatus::IDLE;

    do
    {
        RosInterface::spin_some();

        status = tree.tickRoot();

        if (status != last_status)
        {
            switch (status)
            {
            case BT::NodeStatus::IDLE:
                std::cout << "[MainTree] Status: IDLE" << std::endl;
                break;
            case BT::NodeStatus::RUNNING:
                std::cout << "[MainTree] Status: RUNNING" << std::endl;
                break;
            case BT::NodeStatus::SUCCESS:
                std::cout << "[MainTree] Status: SUCCESS" << std::endl;
                break;
            case BT::NodeStatus::FAILURE:
                std::cout << "[MainTree] Status: FAILURE" << std::endl;
                break;
            }

            last_status = status;
        }

    } while (status == BT::NodeStatus::IDLE || status == BT::NodeStatus::RUNNING);

    return status;
}

int main(int argc, char **argv)
{
    RosInterface::init("Coordinator", argc, argv);

    RosInterface::get_node_handle()->declare_parameter("main_tree_path", rclcpp::PARAMETER_STRING);
    RosInterface::get_node_handle()->declare_parameter("groot_palette_path", rclcpp::PARAMETER_STRING);

    std::string main_tree_path;
    std::string groot_palette_path;

    try
    {
        main_tree_path = RosInterface::get_node_handle()->get_parameter("main_tree_path").as_string();
        groot_palette_path = RosInterface::get_node_handle()->get_parameter("groot_palette_path").as_string();
        std::cout << main_tree_path << std::endl;
        std::cout << groot_palette_path << std::endl;
    }
    catch (const rclcpp::exceptions::ParameterNotDeclaredException &e)
    {
        std::cerr << e.what() << " not declared\n";
    }

    BT::Tree tree = create_tree(main_tree_path, groot_palette_path);

    std::unique_ptr<BT::PublisherZMQ> zmq_publisher = std::make_unique<BT::PublisherZMQ>(tree);

    run_tree(tree);

    zmq_publisher.reset();

    RosInterface::shutdown();

    return 0;
}
