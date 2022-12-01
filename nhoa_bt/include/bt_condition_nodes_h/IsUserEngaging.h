#ifndef IsUserEngaging_H_INCLUDED__
#define IsUserEngaging_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_hri.h>

/* TODO: Here should be implemented all the functionalities of:
         - Body detection -> pyhri (ROS4HRI) */

class IsUserEngaging : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_hri* hri_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 

    IsUserEngaging(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(handle_hri* input_hri)
    {
      std::cout << "### Initializing IsUserEngaging! ###" << std::endl;
      hri_    = input_hri;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user is engaging.
    bool isUserEngaging();
};
#endif //IsUserEngaging_H_INCLUDED__
