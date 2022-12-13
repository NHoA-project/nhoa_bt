#include <IsUserAnswered.h>
#include  <iostream>

BT::NodeStatus IsUserAnswered::tick() 
{

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
            // Get the Blackboard input arguments.
            auto conversation_mode    = getInput<std::string>("_conversation_mode");
            if(conversation_mode.value().empty())
            {
                throw BT::RuntimeError("error reading port [conversation_mode]:", conversation_mode.error());
            }
            // ==========
            else
            {
                success = isUserAnswered(conversation_mode.value());
                waiting = false;
            }
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
bool IsUserAnswered::isUserAnswered(const std::string &conversation_mode)
{
    while(!hri_->live_speech_flag_)
    {}

    if( hri_->speech_.find("sí") != std::string::npos ) // "Yes" found.
    {
        std::cout << "YES found!" << std::endl;
        setOutput("user_answered_", "true");
        success_ = true;
    }
    else if( hri_->speech_.find("no") != std::string::npos ) // "No" found.
    {
        std::cout << "NO found!" << std::endl;
        setOutput("user_answered_", "false");
        if( conversation_mode.compare("questionnaire") == 0)
            success_ = false;
        else
            success_ = true;
    }
    hri_->live_speech_flag_ = false;
    return success_;
}