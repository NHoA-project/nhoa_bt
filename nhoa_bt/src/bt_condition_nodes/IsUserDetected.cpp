#include <IsUserDetected.h>
#include  <iostream>

BT::NodeStatus IsUserDetected::tick() {
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

    // std::this_thread::sleep_for(std::chrono::milliseconds(500));
    // return  BT::NodeStatus::SUCCESS;
}
