#include <IsQuestionnaireRequested.h>
#include  <iostream>

BT::NodeStatus IsQuestionnaireRequested::tick() 
{
    BT::NodeStatus success;

    BT::Optional<std::string> input = getInput<std::string>("_questionnaire_requested");
    if(input.has_value()){
        if (input.value().compare("false") == 0) 
        { 
            std::cout << "Questionnaire NOT requested! Going to T6 ..." << std::endl;
            success = BT::NodeStatus::FAILURE;            
        } 
        else if (input.value().compare("true") == 0)
        {
            std::cout << "Questionnaire requested!" << std::endl;
            success = BT::NodeStatus::SUCCESS;
        } 
    }
    else
    {
        std::cout << name() << "There is no information on the blackboard. It is assumed that questionnaire is NOT requesrted."<< std::endl;
        success = BT::NodeStatus::FAILURE;  
    }

    return success;
}