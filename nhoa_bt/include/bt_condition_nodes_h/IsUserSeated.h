#ifndef IsUserSeated_H_INCLUDED__
#define IsUserSeated_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_scene.h>

/* TODO: UPO scene segmentation is_user_sat condition check */

class IsUserSeated : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_scene* scene_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 

    IsUserSeated(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(handle_scene* input_scene)
    {
      std::cout << "### Initializing IsUserSeated! ###" << std::endl;
      scene_    = input_scene;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user is seated.
    bool isUserSeated();
};
#endif //IsUserSeated_H_INCLUDED__
