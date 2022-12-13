#include <handle_hri.h>

// C++ standard headers
#include <exception>
#include <string>

// Boost headers
#include <boost/shared_ptr.hpp>

// ROS headers
#include <ros/ros.h>
#include <actionlib/client/simple_action_client.h>

// PAL includes
#include <hri_msgs/IdsList.h>
#include <hri_msgs/IdsMatch.h>
#include <hri_msgs/EngagementLevel.h>
#include <hri_msgs/FacialLandmarks.h>

// C++ standard headers
#include <cstdlib>

// =====================================

handle_hri::handle_hri(ros::NodeHandle    *nodehandle):  nh_(*nodehandle)
{
  handle_hri::init();
}

// ###################################

void handle_hri::candidates_matches_callback(const hri_msgs::IdsMatch&  ids_match)
{
  // Check id matches.
  // mutex_.lock();
  match_confidence_ = ids_match.confidence;
  id1_ = ids_match.id1;
  id1_type_ = handle_hri::set_id_type(ids_match.id1_type);
  id2_ = ids_match.id2;
  id2_type_ = handle_hri::set_id_type(ids_match.id2_type);
  // mutex_.unlock();
}

void handle_hri::engagement_level_callback(const hri_msgs::EngagementLevel&  engagement)
{
  handle_hri::set_engagement_level(engagement.level);
}

void handle_hri::init()
{
  std::cout << "Initializing handle_hri ..." << std::endl;

  // Initialize user_input subscriber. 
  candidate_matches_sub_    = nh_.subscribe("/humans/candidate_matches", 1, &handle_hri::candidates_matches_callback, this);
  live_speech_sub_          = nh_.subscribe("/humans/voices/anonymous_speaker/speech", 1, &handle_hri::live_speech_callback, this);
  // tracked_bodies_sub_       = nh_.subscribe("/humans/bodies/tracked", 1, &handle_hri::tracked_bodies_callback, this);
  // tracked_faces_sub_        = nh_.subscribe("/humans/faces/tracked", 1, &handle_hri::tracked_faces_callback, this);
  // tracked_persons_sub_      = nh_.subscribe("/humans/persons/tracked", 1, &handle_hri::tracked_persons_callback, this);

  // Initialize variables.
  match_confidence_threshold_ = nh_.param("ids_match_confidence_threshold", 0.5);
  smile_score_threshold_      = nh_.param("smile_score_threshold", 0.5);

  std::cout << "handle_hri initialized!" << std::endl;
}

void handle_hri::live_speech_callback(const hri_msgs::LiveSpeech&  speech)
{
  std::cout << "Live speech -> " << speech.final << std::endl; 
  speech_           = speech.final;
  if(speech_.find("sí") != std::string::npos || speech_.find("no") != std::string::npos)
    live_speech_flag_ = true;
}

void handle_hri::matched_id_subscriber_loop()
{
  while((body_id_.empty()) || (face_id_.empty()) || (person_id_.empty())) // TODO: Check that condition.
  {
    std::cout << "Tracked ID is empty." << std::endl;
  }

  // Set the matched id (body, face and person).
  handle_hri::set_id_matches(id1_, id1_type_);
  handle_hri::set_id_matches(id2_, id2_type_);

  engagement_level_sub_ = nh_.subscribe("/humans/person/" + person_id_ + "/engagement_status", 1, &handle_hri::engagement_level_callback, this);
  smile_score_sub_      = nh_.subscribe("/humans/faces/" + face_id_ + "/smile_score", 1, &handle_hri::smile_score_callback, this);

}

void handle_hri::set_id_matches(const std::string &id,
                                const std::string &id_type)
{
  if(match_confidence_ >= match_confidence_threshold_)
  {
    mutex_.lock();
    if(id_type.compare("body") == 0)
    {
      body_id_ = id;
    }
    else if(id_type.compare("face") == 0)
    {
      face_id_ = id;
    }
    else if(id_type.compare("person") == 0)
    {
      person_id_ = id;
    }
    mutex_.unlock();
  }
}

void handle_hri::set_engagement_level(const uint &level)
{
  switch (level)
  {
  case 0:
      engagement_level_ = "unknown";
    break;
  case 1:
      engagement_level_ = "disengaged";
    break;
  case 2:
      engagement_level_ = "engaging";
    break;
  case 3:
      engagement_level_ = "engaged";
    break;
  case 4:
      engagement_level_ = "disengaging";
    break;
  }
}

std::string handle_hri::set_id_type(const int &id_type)
{
  std::string type;

  switch (id_type)
  {
  case 0:
      type = "unset";
    break;
  case 1:
      type = "person";
    break;
  case 2:
      type = "face";
    break;
  case 3:
      type = "body";
    break;
  case 4:
      type = "voice";
    break;
  }
  return type;
}

void handle_hri::smile_score_callback(const std_msgs::Float32::ConstPtr&  score)
{
  std::cout << "Smile score -> " << score->data << std::endl; 
  smile_score_ = score->data;
}


void handle_hri::tracked_bodies_callback(const hri_msgs::IdsList&  ids_list)
{
  // Debug.
  std::cout << "Body User ID -> " << ids_list.ids[0] << std::endl; 
  if( !(body_id_.compare(ids_list.ids[0]) == 0) )
  {
    body_id_ = ids_list.ids[0]; 
  }
}

void handle_hri::tracked_faces_callback(const hri_msgs::IdsList&  ids_list)
{
  // Debug.
  std::cout << "Face User ID -> " << ids_list.ids[0] << std::endl; 
  if( !(face_id_.compare(ids_list.ids[0]) == 0) )
  {
    face_id_ = ids_list.ids[0]; 
    handle_hri::matched_id_subscriber_loop();
  }
}

void handle_hri::tracked_persons_callback(const hri_msgs::IdsList&  ids_list)
{
  // Debug.
  std::cout << "Person User ID -> " << ids_list.ids[0] << std::endl; 
  if( !(person_id_.compare(ids_list.ids[0]) == 0) )
  {
    person_id_ = ids_list.ids[0]; 
    handle_hri::matched_id_subscriber_loop();
  }
}