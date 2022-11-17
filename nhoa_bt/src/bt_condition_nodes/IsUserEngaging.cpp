#include <IsUserEngaging.h>
#include  <iostream>

BT::NodeStatus IsUserEngaging::tick() {
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
            return BT::NodeStatus::SUCCESS;
        } 
        else if (input.compare("3") == 0) 
        { 
            std::cout << "Engagement Level -> '3' (ENGAGED)" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
        else if (input.compare("4") == 0) 
        { 
            std::cout << "Engagement Level -> '4' (DISENGAGING)" << std::endl;
            return BT::NodeStatus::FAILURE;
        } 
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // return  BT::NodeStatus::SUCCESS;
}
