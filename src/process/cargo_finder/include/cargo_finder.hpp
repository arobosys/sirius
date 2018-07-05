#ifndef __CARGO_FINDER_CLASS_
#define __CARGO_FINDER_CLASS_

#include <ros/ros.h>
#include <std_msgs/String.h>
#include <process_interface/process_interface.hpp>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include <tf/transform_listener.h>
#include <move_base_msgs/MoveBaseAction.h>
#include "math.h"
#include <fiducial_msgs/FiducialTransformArray.h>


typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class CargoFinder {

private:;
    void sendFeedback(std::map<std::string, std::string> &result);

public:;

    void fidCallback(const fiducial_msgs::FiducialTransformArray& fid);

    void goalCallback(const interface::ProcessInterface::Parameters &params) ;
    void preemtCallback();

    /**
     * Class constructor
     */
    CargoFinder(ros::NodeHandle &handle, int argc, char **argv);

protected:;
    std::shared_ptr<interface::ProcessInterface> rosLinkClientPtr_ = nullptr;
    ros::NodeHandle handle_;
  ros::Subscriber fid_sub_;
  fiducial_msgs::FiducialTransform fid105, fid110;
  bool flag1, flag2, flag_positioning_;
  bool  goal_active;
  std::list<tf::Vector3> queue_;
};

#endif //__CARGO_FINDER_CLASS_
