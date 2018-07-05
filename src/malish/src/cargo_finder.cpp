// Created by Georgy on 11.04.18.
/*!
 * \file cargo_finder.cpp
 *
 * Listen to transform of markers and publish the goal position.
 *
 * \author Ostroumov Georgy
 *
*/

#include "ros/ros.h"
#include <tf/transform_listener.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <tf/tf.h>
#include <geometry_msgs/Quaternion.h>
#include "math.h"
#include <fiducial_msgs/FiducialTransformArray.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class cargo_find {
  public:;
    cargo_find() {
        ros::NodeHandle nh_;
        ros::Rate rate(5.0);
        fid_sub_ =  nh_.subscribe("/fiducial_transforms", 10, &cargo_find::fidCallback, this);
        flag_positioning_ = true;

    }
  void fidCallback(const fiducial_msgs::FiducialTransformArray& fid)
  {
      MoveBaseClient ac("move_base",true);
      while(!ac.waitForServer(ros::Duration(3.0))){
          ROS_INFO("Waiting for the move_base action server to come up");
      }
      ROS_INFO("SIZE: %zu", fid.transforms.size() );
      if (fid.transforms.size() == 2) {
          fid105 = fid.transforms[0];
          ROS_INFO("ID 105:%i",fid105.fiducial_id );
          fid110 = fid.transforms[1];
          ROS_INFO("ID 110:%i",fid110.fiducial_id );
          tf::Quaternion q_rot(-0.5,0.5,-0.5,0.5);
          tf::Vector3 fid110pos(fid110.transform.translation.x,fid110.transform.translation.y,fid110.transform.translation.z);
          tf::Vector3 fid105pos(fid105.transform.translation.x,fid105.transform.translation.y,fid105.transform.translation.z);
          fid110pos = tf::quatRotate(q_rot,fid110pos);
          fid105pos = tf::quatRotate(q_rot,fid105pos);
          tf::Quaternion quat110, quat105, quatmid;
          tf::quaternionMsgToTF(fid105.transform.rotation, quat105);
          tf::quaternionMsgToTF(fid110.transform.rotation, quat110);
          //quat110 = q_rot*quat110;
          //quat105 = q_rot*quat105;
          double start_dist = 0.75, fin_dist = 0.3;
          flag1 = true;
          flag2 = true;
          move_base_msgs::MoveBaseGoal goal;

          if(flag1 && flag2)
          {
              goal.target_pose.header.frame_id = "zed_left_camera";
              goal.target_pose.header.stamp = ros::Time::now();

              goal.target_pose.pose.position.x = (fid105pos.x() + fid110pos.x())/2;
              goal.target_pose.pose.position.y = (fid105pos.y() + fid110pos.y())/2;
              goal.target_pose.pose.position.z = 0;//(fid105pos.z() + fid110pos.z())/2;

              quatmid = quat105.slerp(quat110, 0.5);

              tf::Quaternion q_rot2, q_new;
              double r=0, p=0, y=atan2((fid110pos.x()-fid105pos.x()), -(fid110pos.y()-fid105pos.y()));  // Rotate the previous pose by 180* about Z

              if((y<-1.57) or (y > 1.57))

              {
                  y = y+3.14;
              }
              //q_rot2 = tf::Quaternion(-0.5,0.5,-0.5,-0.5);
              q_rot2 = tf::createQuaternionFromRPY(r, p, y);
              q_new =  q_rot2;  // Calculate the new orientation
              q_new.normalize();
              geometry_msgs::Quaternion qMsg;
              tf::quaternionTFToMsg(q_new, qMsg);
              if (flag_positioning_) {
                  goal.target_pose.pose.position.x = goal.target_pose.pose.position.x - cos(y) * start_dist;
                  goal.target_pose.pose.position.y = goal.target_pose.pose.position.y - sin(y) * start_dist;
                  flag_positioning_= false;
              } else
              {
                  goal.target_pose.pose.position.x = goal.target_pose.pose.position.x + cos(y) * fin_dist;
                  goal.target_pose.pose.position.y = goal.target_pose.pose.position.y + sin(y) * fin_dist;
                  flag_positioning_ = true;
              }
              goal.target_pose.pose.orientation = qMsg;
              ROS_INFO("Got coordinate: %f, %f, %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z);
              ROS_INFO("Got quaternion: x:%f, y:%f, z:%f,w: %f", goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
              ROS_INFO("Sending goal to move_base");
              ac.sendGoal(goal);
              ac.waitForResult();
              if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                  ROS_INFO("Malish, the robot moved to goal");
              } else {
                  ROS_INFO("The robot failed to move to goal for some reason");
                  ros::Duration(1).sleep();
              }
          }
      }
  }
  private:;
    ros::Subscriber fid_sub_;
    fiducial_msgs::FiducialTransform fid105, fid110;
    bool flag1, flag2, flag_positioning_;

};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "fid_tf_node");

    cargo_find cargo_finder;

    ros::spin();

}
