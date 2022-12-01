#ifndef IsUserDetected_H_INCLUDED__
#define IsUserDetected_H_INCLUDED__

#include <behaviortree_cpp_v3/behavior_tree.h>
#include <behaviortree_cpp_v3/bt_factory.h>

// NHOA_BT INCLUDES
#include <handle_hri.h>


/* TODO: Here should be implemented all the functionalities of:
         - Body detection -> hri_fullbody (ROS4HRI) 
         - Face detection -> hri_facedetect (ROS4HRI) */

class IsUserDetected : public BT::ConditionNode
{
  public:

    // Shared program resources.
    handle_hri* hri_;

    // Boolean stuff.
    bool success_                 = false;

    // =================================================== 
    
    IsUserDetected(const std::string& name) :
        BT::ConditionNode(name, {})
    {
    }

    void init(handle_hri* input_hri)
    {
      std::cout << "### Initializing IsUserDetected! ###" << std::endl;
      hri_    = input_hri;
    }

    // You must override the virtual function tick()
    BT::NodeStatus tick() override;

    // =================================================== 
    // Check if user is detected.
    bool isUserDetected();
};
#endif //IsUserDetected_H_INCLUDED__
