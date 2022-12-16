// BT INCLUDES
#include <behaviortree_cpp_v3/loggers/bt_cout_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_minitrace_logger.h>
#include <behaviortree_cpp_v3/loggers/bt_file_logger.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// ROS INCLUDES
#include <ros/ros.h>
#include <ros/package.h>

// STD INCLUDES
#include <vector>
#include <string>

// ZMQ INCLUDES
// #ifdef ZMQ_FOUND
#include "behaviortree_cpp_v3/loggers/bt_zmq_publisher.h"
// #endif

#include <fstream>

#define MANUAL_STATIC_LINKING

#ifdef MANUAL_STATIC_LINKING

// ACTION NODES
#include <CancelNavigationGoal.h>
#include <CheckSmileScore.h>
#include <DrawUserAttention.h>
#include <ExecuteConversation.h>
#include <ExecuteHeadMotion.h>
#include <ExecuteGUI.h>
#include <ExecuteNavigation.h>
#include <ExecuteUperbodyMotion.h>
#include <ExecuteVoiceCmd.h>
#include <UploadUserInput.h>

// CONDITION NODES
#include <IsApproximationReached.h>
#include <IsBottleOnTable.h>
#include <IsUserAnswered.h>
#include <IsUserDetected.h>
#include <IsUserEngaged.h>
#include <IsUserEngaging.h>
#include <IsUserSeated.h>

// RECYCLA FILES INCLUDES
#include <handle_gui.h>
#include <handle_hri.h>
#include <handle_voice.h>
// #include <handle_scene.h>
#include <plan_head_motion.h>
#include <plan_motion.h>
#include <plan_navigation.h>

#endif

using namespace BT;

int main(int argc, char **argv)
{
    // Initialize ROS
    ros::init(argc, argv, "nhoa_bt_demo");
    ros::NodeHandle nh;

    ros::AsyncSpinner spinner(0);
    spinner.start();

    // Initialize resources [TODO: Add new instances here].
    handle_gui          gui(&nh);
    handle_hri          hri(&nh);
    handle_voice        voice(&nh);
    handle_scene        scene(&nh);
    plan_head_motion    head_motion(&nh);
    plan_motion         motion(&nh);
    plan_navigation     navigation(&nh);

    // Load xml BT file
    std::string default_path = ros::package::getPath("nhoa_bt");
    std::string default_local_path = "/include/nhoa_bt/BT_XML/NHoADemoBT.xml";

    default_path += default_local_path;
    std::string path = nh.param("bt_xml_path", default_path); //If param ok, the path is completely defined (package+local path), if not default path is calculated
    ROS_INFO_STREAM("Path: " << path);
    std::ifstream ifs(path);
    if (!ifs.is_open())
    {
        ROS_ERROR("File not open");
        return 0;
    }
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    ifs.close();
    std::string xml_text = buffer.str();


    // We use the BehaviorTreeFactory to register our custom nodes
    BehaviorTreeFactory factory;
    
#ifdef MANUAL_STATIC_LINKING
    // Note: the name used to register should be the same used in the XML.
    // Note that the same operations could be done using DummyNodes::RegisterNodes(factory)

    // The recommended way to create a Node is through inheritance.
    // Even if it requires more boilerplate, it allows you to use more functionalities
    // like ports (we will discuss this in future tutorials).

    std::cout << "BT_CONTROL: Starting the node registration" << std::endl;

    // ACTION NODES (BT full implemented).
    factory.registerNodeType<CancelNavigationGoal>("CancelNavigationGoal");
    factory.registerNodeType<CheckSmileScore>("CheckSmileScore");
    factory.registerNodeType<DrawUserAttention>("DrawUserAttention");
    factory.registerNodeType<ExecuteConversation>("ExecuteConversation");
    factory.registerNodeType<ExecuteGUI>("ExecuteGUI");
    factory.registerNodeType<ExecuteHeadMotion>("ExecuteHeadMotion");
    factory.registerNodeType<ExecuteNavigation>("ExecuteNavigation");
    factory.registerNodeType<ExecuteUperbodyMotion>("ExecuteUperbodyMotion");
    factory.registerNodeType<ExecuteVoiceCmd>("ExecuteVoiceCmd");
    factory.registerNodeType<UploadUserInput>("UploadUserInput");

    // ACTION NODES (Class + BT wrapped functionalities).    
    // factory.registerSimpleAction("ResetWrist3",
    //                              std::bind(&wrist_3_arm_movements::ResetWrist3, &wrist3));

    // CONDITION NODES (BT full implemented).
    // factory.registerNodeType<ConditionNodeFrame>("ConditionNodeFrame");
    factory.registerNodeType<IsApproximationReached>("IsApproximationReached");
    factory.registerNodeType<IsBottleOnTable>("IsBottleOnTable");
    factory.registerNodeType<IsUserAnswered>("IsUserAnswered");
    factory.registerNodeType<IsUserDetected>("IsUserDetected");
    factory.registerNodeType<IsUserEngaged>("IsUserEngaged");
    factory.registerNodeType<IsUserEngaging>("IsUserEngaging");
    factory.registerNodeType<IsUserSeated>("IsUserSeated");


    // CONDITION NODES (Class + BT wrapped functionalities).
    // factory.registerSimpleCondition("IsGrasping",
    //                                 std::bind(&gripper_status::IsGrasping, &gripper_status));

    std::cout << "BT_CONTROL: All nodes registered" << std::endl;

#else
    // Load dynamically a plugin and register the TreeNodes it contains
    // it automated the registering step.
    //factory.registerFromPlugin("./libdummy_nodes_dyn.so");
#endif

    // Trees are created at deployment-time (i.e. at run-time, but only once at the beginning).
    // The currently supported format is XML.
    // IMPORTANT: when the object "tree" goes out of scope, all the TreeNodes are destroyed
    auto tree = factory.createTreeFromText(xml_text);
    
    // This logger prints state changes on console
    StdCoutLogger logger_cout(tree);

    // This logger stores the execution time of each node
    MinitraceLogger logger_minitrace(tree, "nhoa_demo_bt_trace.json");

    // This logger saves state changes on file
    FileLogger logger_file(tree, "nhoa_demo_bt_trace.fbl");

// #ifdef ZMQ_FOUND

    // This logger publish status changes using ZeroMQ. Used by Groot
    PublisherZMQ publisher_zmq(tree);
    std::cout << "BT COMMUNICATION: ZMQ found" << std::endl;
    
// #endif

    std::cout << "Print tree recursively!" << std::endl;
    printTreeRecursively(tree.rootNode());

    // Iterate through all the nodes and call init() if it is an Action_B
    for (auto &node : tree.nodes)
    {
        // BT Action nodes.
        if(auto CancelNavigationGoal_node = dynamic_cast<CancelNavigationGoal *>(node.get()))
        {
            CancelNavigationGoal_node->init(&navigation); 
        }
        else if(auto CheckSmileScore_node = dynamic_cast<CheckSmileScore *>(node.get()))
        {
            CheckSmileScore_node->init(&hri); 
        }
        else if(auto ExecuteGUI_node = dynamic_cast<ExecuteGUI *>(node.get()))
        {
            ExecuteGUI_node->init(&nh, &gui); 
        }
        else if(auto ExecuteHeadMotion_node = dynamic_cast<ExecuteHeadMotion *>(node.get()))
        {
            ExecuteHeadMotion_node->init(&nh, &head_motion); 
        }
        else if(auto ExecuteUperbodyMotion_node = dynamic_cast<ExecuteUperbodyMotion *>(node.get()))
        {
            ExecuteUperbodyMotion_node->init(&nh, &motion);
        }
        else if (auto ExecuteNavigation_node = dynamic_cast<ExecuteNavigation *>(node.get()))
        {
            ExecuteNavigation_node->init(&nh, &navigation);
        }
        else if (auto ExecuteVoiceCmd_node = dynamic_cast<ExecuteVoiceCmd *>(node.get()))
        {
            ExecuteVoiceCmd_node->init(&nh, &voice);
        }
        else if (auto UploadUserInput_node = dynamic_cast<UploadUserInput *>(node.get()))
        {
            UploadUserInput_node->init(&gui, &hri);
        }
        // --------------
        // BT Condition nodes.
        else if (auto IsApproximationReached_node = dynamic_cast<IsApproximationReached *>(node.get()))
        {
            IsApproximationReached_node->init(&navigation);
        }
        else if (auto IsBottleOnTable_node = dynamic_cast<IsBottleOnTable *>(node.get()))
        {
            IsBottleOnTable_node->init(&scene);
        }
        else if (auto IsUserAnswered_node = dynamic_cast<IsUserAnswered *>(node.get()))
        {
            IsUserAnswered_node->init(&hri);
        }
        else if (auto IsUserDetected_node = dynamic_cast<IsUserDetected *>(node.get()))
        {
            IsUserDetected_node->init(&hri, &head_motion);
        }
        else if (auto IsUserEngaging_node = dynamic_cast<IsUserEngaging *>(node.get()))
        {
            IsUserEngaging_node->init(&hri);
        }
        else if (auto IsUserEngaged_node = dynamic_cast<IsUserEngaged *>(node.get()))
        {
            IsUserEngaged_node->init(&hri);
        }
        else if (auto IsUserSeated_node = dynamic_cast<IsUserSeated *>(node.get()))
        {
            IsUserSeated_node->init(&scene);
        }
    }

    const bool LOOP = ( argc == 2 && strcmp( argv[1], "loop") == 0);

    do
    {
        NodeStatus status = NodeStatus::RUNNING;
        // Keep on ticking until you get either a SUCCESS or FAILURE state
        while( status == NodeStatus::RUNNING)
        {
            status = tree.tickRoot();
        }
    }
    while(LOOP);

    return 0;
}