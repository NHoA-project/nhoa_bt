#include <IsUserDetected.h>
#include  <iostream>

BT::NodeStatus IsUserDetected::tick() {
    // TODO: HARDCODED VERSION.
    std::string input;
    while (true) {
        std::cout << name() << ": Is user detected? Select 's' (success) or 'f'(failure)" << std::endl;
        std::cin >> input;
        if (input.compare("s") == 0) 
        { 
            std::cout << "User is detected!" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else if(input.compare("f") == 0) 
        {
            std::cout << "User is NOT detected!" << std::endl;
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
    //         if(isUserDetected())
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
bool IsUserDetected::isUserDetected()
{
    if( !(hri_->body_id_.empty()) || 
        !(hri_->face_id_.empty()) || 
        !(hri_->person_id_.empty()) ) // TODO: HRI ID detection
    // if( hri_->engagement_level_.compare("engaged") == 0 ) // TODO: EngagementLevel
    {
        success_ = true;
    }
    else
    {
        success_ = false;
    }
    return success_;
}
