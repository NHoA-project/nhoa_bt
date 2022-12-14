#include <IsApproximationReached.h>
#include  <iostream>

BT::NodeStatus IsApproximationReached::tick() {
    // TODO: CLEAN VERSION.
    std::string input;
    bool waiting = true;
    bool success = false;
    while (waiting) 
    {
        std::cout << name() << ": Select 's' (start) or 'p'(pause)" << std::endl;
        std::cin >> input;
        if (input.compare("s") == 0) 
        { 
            
            success = isApproximationReached();

            waiting = false;
        } 
    }

    if(success)
    {
        return BT::NodeStatus::SUCCESS;
    }
    else
    {
        return BT::NodeStatus::FAILURE;
    }
}

// ####################
// Additional functions.
bool IsApproximationReached::isApproximationReached()
{
    return navigation_->check_approach_distance();
}