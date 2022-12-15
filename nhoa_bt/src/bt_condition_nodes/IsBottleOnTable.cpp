#include <IsBottleOnTable.h>
#include  <iostream>

BT::NodeStatus IsBottleOnTable::tick() {
    // // TODO: HARDCODED VERSION.
    // std::string input;
    // while (true) {
    //     std::cout << name() << ": Is bottle on table? Select 's' (success) or 'f'(failure)" << std::endl;
    //     std::cin >> input;
    //     if (input.compare("s") == 0) 
    //     { 
    //         std::cout <<  Bottle is on table!" << std::endl;
    //         return BT::NodeStatus::SUCCESS;
    //     } 
    //     else if(input.compare("f") == 0) 
    //     {
    //         std::cout << "Bottle is not on table! << std::endl;
    //         return BT::NodeStatus::FAILURE;
    //     } 
    // }

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
            if(isBottleOnTable())
                success = true;

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
bool IsBottleOnTable::isBottleOnTable()
{
    if(scene_->scene_digest_msg_.bottle_on_table)
    {
        std::cout << "Is bottle on table? -> " << "True" << std::endl;
        success_ = true;
    }
    else
    {
        std::cout << "Is bottle on table? -> " << "False" << std::endl;
        success_ = false;
    }
    return success_;
}