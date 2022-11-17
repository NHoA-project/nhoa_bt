#include <IsFirstUnfoldingPointFound.h>
#include  <iostream>


BT::NodeStatus IsFirstUnfoldingPointFound::tick() {
    std::string input;
    while (true) {
        std::cout << name() << ": Select 's' (success) or 'f'(failure)" << std::endl;
        std::cin >> input;
        if (input.compare("s") == 0) 
        { 
            return BT::NodeStatus::SUCCESS;
        } 
        else if(input.compare("f") == 0) 
        {
            return BT::NodeStatus::FAILURE;
        } 
    }

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // return  BT::NodeStatus::SUCCESS;
}
