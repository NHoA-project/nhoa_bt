#include <IsApproximationReached.h>
#include  <iostream>

BT::NodeStatus IsApproximationReached::tick() {
    // TODO: HARDCODED VERSION.
    std::string input;
    while (true) {
        std::cout << name() << ": Is approximation reached? Select 's' (success) or 'f'(failure)" << std::endl;
        std::cin >> input;
        if (input.compare("s") == 0) 
        { 
            std::cout <<  "Approximation is reached!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else if(input.compare("f") == 0) 
        {
            std::cout << "Approximation is NOT reached!" << std::endl;
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
            
    //         success = isApproximationReached();

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
bool IsApproximationReached::isApproximationReached()
{
    return navigation_->check_approach_distance();
}