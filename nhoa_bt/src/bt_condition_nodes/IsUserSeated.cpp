#include <IsUserSeated.h>
#include  <iostream>

BT::NodeStatus IsUserSeated::tick() {

    // TODO: HARDCODED VERSION.
    std::string input;
    while (true) {
        std::cout << name() << ": Is user sat? Select 's' (success) or 'f'(failure)" << std::endl;
        std::cin >> input;
        if (input.compare("s") == 0) 
        { 
            std::cout << "User is sat!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else if(input.compare("f") == 0) 
        {
            std::cout << "User is NOT sat!" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
    }

    // // TODO: CLEAN VERSION.
    // std::string input;
    // bool waiting = true;
    // bool success = false;
    // while (waiting) 
    // {
    //     std::cout << name() << ": Select 's' (start) or 'p'(pause)" << std::endl;
    //     std::cin >> input;
    //     if (input.compare("s") == 0) 
    //     { 
    //         if(isUserSeated())
    //             success = true;

    //         waiting = false;
    //     } 
    // }

    // if(success)
    // {
    //     return BT::NodeStatus::SUCCESS;
    // }
    // else
    // {
    //     return BT::NodeStatus::FAILURE;
    // }
}

// ####################
// Additional functions.
bool IsUserSeated::isUserSeated()
{
    if(scene_->scene_digest_msg_.person_sitting_on_chair)
    {
        std::cout << "Is person sitting on chair? -> " << "True" << std::endl;
        success_ = true;
    }
    else
    {
        std::cout << "Is person sitting on chair? -> " << "False" << std::endl;
        success_ = false;
    }
    return success_;
}