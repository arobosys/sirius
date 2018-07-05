#include <map>
#include <unordered_map>
#include "GoalSender.hpp"
#include <math.h>
#include <string>
#include "file_coordinates.h"

/**
\file
\brief Header File
Class receiving Goal from Process Layer, read coordinates from file and sand goal to move_base
* N - decimal number of single coordinate
*/

GoalSender::GoalSender(ros::NodeHandle &handle, int argc, char **argv) {

}

std::map<std::string, std::string> GoalSender::parseTransforms(const std::map<std::string, std::string> &keyToValue) {

    std::map<std::string, std::string> result;
    //std::string goal1_string = keyToValue.at(GOAL1);
    int curr_goal = 0;
#define FILE_NAME "route1.txt"

    std::string s_file_name(FILE_NAME);
    std::vector<POINT> vec_points_from_file;
    std::unique_ptr<FileCoordinates> fCoordinates(new FileCoordinates);

    if(!fCoordinates->GetCoordinatesVectorFromFile(s_file_name, vec_points_from_file)){
        ROS_WARN("Route file not found");
        result["err"] = "file_missed";
        return result;
        }
    //tell the action client that we want to spin a thread by default
    MoveBaseClient ac("move_base", true);

    //wait for the action server to come up
    while(!ac.waitForServer(ros::Duration(10.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    for(auto wp = keyToValue.begin(); wp != keyToValue.end(); ++wp) {
        if (wp->first == "WP")
            for(auto it = vec_points_from_file.begin(); it != vec_points_from_file.end(); ++it)
                if(std::stoul(wp->second.c_str()) == it->num )
                {///goal received
                    ROS_INFO("Received goal #%lu : X : %.2lf , Y : %.2lf , Q : %.2lf",it->num, it->x, it->y, it->ang);
                    curr_goal = it->num;
                    move_base_msgs::MoveBaseGoal goal;

                    //The goal_(i+1)
                    goal.target_pose.header.frame_id = "map";
                    goal.target_pose.header.stamp = ros::Time::now();

                    goal.target_pose.pose.position.x = it->x;
                    goal.target_pose.pose.position.y = it->y;

                    double radians = it->ang * (M_PI / 180.0);
                    tf::Quaternion quaternion;
                    quaternion = tf::createQuaternionFromYaw(radians);
                    geometry_msgs::Quaternion qMsg;
                    tf::quaternionTFToMsg(quaternion, qMsg);
                    goal.target_pose.pose.orientation = qMsg;

                    int retry = 0;
                    do {
                        if (retry)
                            ROS_INFO("Retry #%d sending a goal %d to move_base", retry, curr_goal );
                        ROS_INFO("Sending goal %d to move_base", curr_goal );
                        ac.sendGoal(goal);

                        ac.waitForResult();

                        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
                            ROS_INFO("Malish, the robot moved to goal %d", curr_goal);
                        } else {
                            ROS_INFO("The robot failed to move to goal %d for some reason", curr_goal );
                            retry++;
                            ros::Duration(1).sleep();
                        }
                    } while ((ac.getState() != actionlib::SimpleClientGoalState::SUCCEEDED));// || _gogogo
                }//goal received
    }//for num_of_goal
    if(curr_goal > 0)
        result["reach"] = "OK";

    return result;
}

void GoalSender::goalCallback(const interface::ProcessInterface::Parameters &params)
{
    auto data = params.key_value;


    typedef std::map<std::string, std::string> map_SS_T;
    map_SS_T keyToValue, result;

    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }

    result = parseTransforms(keyToValue);

    using interface::ProcessInterface;
    ProcessInterface::Result alibResultOk, alibResultFail;
    ProcessInterface::Feedback feedback;
    core_msgs::DataPL dataPL;
    dataPL.command = "execute";
    dataPL.id = rosLinkClientPtr->getId();
    dataPL.type = "feedback";
    core_msgs::KeyValue keyValue;

    for(auto it = result.begin(); it != result.end(); ++it) {
        keyValue.key = it->first;
        keyValue.value = it->second;

        ROS_INFO("Status: %s : %s", keyValue.key.c_str(), keyValue.value.c_str());

        dataPL.states.push_back(keyValue);
    }

    feedback.feedback.push_back(std::move(dataPL));
    rosLinkClientPtr->publishFeedback(std::move(feedback));

    rosLinkClientPtr->setSucceeded(alibResultOk,"OK");
    ROS_INFO("Sended ResultOk");

    // num_goal = std::stod(keyToValue[NUM_GOAL]);
}

void GoalSender::preemtCallback() {}

