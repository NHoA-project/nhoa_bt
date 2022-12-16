#ifndef IsBottleOnTable_H_INCLUDED__
#define IsBottleOnTable_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_scene.h>

/* TODO: UPO scene segmentation is_bottle_on_table condition check */

class IsBottleOnTable : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_scene* scene_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 

    IsBottleOnTable(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(handle_scene* input_scene)
    {
      std::cout << "### Initializing IsBottleOnTable! ###" << std::endl;
      scene_    = input_scene;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if bottle is on table.
    bool isBottleOnTable();
};
#endif //IsBottleOnTable_H_INCLUDED__
