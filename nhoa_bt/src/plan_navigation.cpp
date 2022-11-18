#include <plan_navigation.h>

// ROS headers
#include <ros/ros.h>

// =====================================

plan_navigation::plan_navigation(ros::NodeHandle    *nodehandle):  nh_(*nodehandle)
{
  // Initialize joint state variables.
  plan_navigation::init();
}

// ###################################

void plan_navigation::cook_navigation(const std::vector<double>    &navigation_goal)
{
  // Cooking navigation.
  std::cout << "Cooking navigation ..." << std::endl;
}

void plan_navigation::init()
{
  std::cout << "Initializing plan_navigation ..." << std::endl;
}

bool plan_navigation::set_navigation_goal(const std::vector<double>    &navigation_goal)
{

  plan_navigation::cook_navigation(navigation_goal);

  // Send goal to "navigation client".
  std::cout << "Navigation goal sent!" << std::endl;

  return true;
}