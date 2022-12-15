#ifndef IsUserSat_H_INCLUDED__
#define IsUserSat_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_scene.h>

/* TODO: Here should be implemented all the functionalities of:
         - Body detection -> pyhri (ROS4HRI) */

class IsUserSat : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_scene* scene_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 

    IsUserSat(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(handle_scene* input_scene)
    {
      std::cout << "### Initializing IsUserSat! ###" << std::endl;
      scene_    = input_scene;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user is engaging.
    bool isUserSat();
};
#endif //IsUserSat_H_INCLUDED__
