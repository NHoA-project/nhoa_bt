#ifndef __PLANNING_NAVIGATION_H_INCLUDED__
#define __PLANNING_NAVIGATION_H_INCLUDED__

// ROS headers
#include <ros/ros.h>

class plan_navigation
{
private:
    //vars
    // ROS stuff
    ros::NodeHandle     nh_;

    // =======
    //functions
    // Initializing.
    void init();

    // Cook navigation.
    void cook_navigation(const std::vector<double>    &navigation_goal);

public:
    //vars

    // =======
    //functions
    plan_navigation(ros::NodeHandle    *nodehandle);

    bool set_navigation_goal(const std::vector<double>    &navigation_goal);
};
#endif // __PLANNING_NAVIGATION_H_INCLUDED__