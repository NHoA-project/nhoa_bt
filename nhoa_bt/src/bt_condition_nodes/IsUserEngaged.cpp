#include <IsUserEngaged.h>
#include  <iostream>

BT::NodeStatus IsUserEngaged::tick() {
    // TODO: HARDCODED VERSION.
    std::string input;
    while (true) 
    {
        std::cout << name() << ": Select user engagement level '0' (UNKNOWN), '1' (DISENGAGED), '2' (ENGAGING), '3' (ENGAGED), '4' (DISENGAGING)" << std::endl;
        std::cin >> input;
        if (input.compare("0") == 0) 
        { 
            std::cout << "Engagement Level -> '0' (UNKNOWN)" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
        else if (input.compare("1") == 0) 
        { 
            std::cout << "Engagement Level -> '1' (DISENGAGED)" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
        else if (input.compare("2") == 0) 
        { 
            std::cout << "Engagement Level -> '2' (ENGAGING)" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
        else if (input.compare("3") == 0) 
        { 
            std::cout << "Engagement Level -> '3' (ENGAGED)" << std::endl;
            return BT::NodeStatus::SUCCESS;
        } 
        else if (input.compare("4") == 0) 
        { 
            std::cout << "Engagement Level -> '4' (DISENGAGING)" << std::endl;
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
    //         if(isUserEngaged())
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
bool IsUserEngaged::isUserEngaged()
{
    if( hri_->smile_score_ > hri_->smile_score_threshold_ ) // TODO: SmileScore
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