#include <map>
#include <unordered_map>
#include "cargo_finder.hpp"
#include <math.h>
#include <string>
#include <list>


// Returns mean of N elements
tf::Vector3 push_and_mean_list(std::list<tf::Vector3> & queue, tf::Vector3 val, unsigned int size=5) {
    static unsigned int qsize = size;
    tf::Vector3 mean(0.0, 0.0, 0.0);

    if(queue.size() == qsize) {
        queue.pop_front();
    } else if (queue.size() > qsize) {
        while(queue.size() > qsize) {
            queue.pop_front();
        }
    }

    queue.push_back(val);

    for (std::list<tf::Vector3>::iterator it = queue.begin(); it != queue.end(); ++it) {
        mean += *it;
    }

    mean = mean / queue.size();

    return mean;
}

/**
\file
\brief Header File
Class receiving Goal from Process Layer, read coordinates from file and sand goal to move_base
* N - decimal number of single coordinate
*/
CargoFinder::CargoFinder(ros::NodeHandle &handle, int argc, char **argv) {

    rosLinkClientPtr_ =
            std::make_shared<interface::ProcessInterface>(argc, argv,
                                                          std::bind(&CargoFinder::goalCallback, this, std::placeholders::_1),
                                                          std::bind(&CargoFinder::preemtCallback, this));
    rosLinkClientPtr_->listen();
    handle_ = handle;

    flag_positioning_ = true;
    goal_active = false;

}

void CargoFinder::goalCallback(const interface::ProcessInterface::Parameters &params) {
    std::map<std::string, std::string> keyToValue;

    // Get key-value from tbOption
    auto data = params.key_value;

    for (const auto &a : data) {
        keyToValue[a.key] = a.value;
    }

    ros::master::V_TopicInfo master_topics;
    ros::master::getTopics(master_topics);

    for (auto it = master_topics.begin() ; it != master_topics.end(); it++) {
        const ros::master::TopicInfo& info = *it;
        if (info.name == "/fiducial_transforms"){
            fid_sub_ =  handle_.subscribe("/fiducial_transforms", 10, &CargoFinder::fidCallback, this);
            std::cout << "Topic : " << it - master_topics.begin() << ": " << info.name << " -> " << info.datatype <<       std::endl;
            goal_active = true;
        }
    }
    if (fid_sub_ == nullptr) {
        ROS_WARN("Topic /fiducial_transforms not started");
        std::map<std::string, std::string> result;
        result["err"] = "Failed";
        this->sendFeedback(result);
        interface::ProcessInterface::Result alibResultFail;
        rosLinkClientPtr_->setAborted(alibResultFail);
    }
    //TODO get id from Database
    goal_active = true;
    ROS_INFO("Received a goal from kernel");
}

void CargoFinder::preemtCallback() {

}

//TODO add rosparametres for parking
#define START_DIST (-0.9)
#define FIN_DIST (0.45)
#define ELEMS (5)
void CargoFinder::fidCallback(const fiducial_msgs::FiducialTransformArray& fid)
{
    if(!goal_active)
        return;
    MoveBaseClient ac("move_base",true);

    if(!ac.waitForServer(ros::Duration(3.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
        return;
    }
    ROS_INFO("SIZE: %d", static_cast<int>(fid.transforms.size()) );
    if (fid.transforms.size() == 2) {
        //TODO id allocation and checking of FID id
        fid105 = fid.transforms[0];
        ROS_INFO("ID 105:%d",fid105.fiducial_id );
        fid110 = fid.transforms[1];
        ROS_INFO("ID 110:%d",fid110.fiducial_id );
        tf::Quaternion q_rot(-0.5,0.5,-0.5,0.5);
        tf::Vector3 fid110pos(fid110.transform.translation.x,fid110.transform.translation.y,fid110.transform.translation.z);
        tf::Vector3 fid105pos(fid105.transform.translation.x,fid105.transform.translation.y,fid105.transform.translation.z);
        fid110pos = tf::quatRotate(q_rot,fid110pos);
        fid105pos = tf::quatRotate(q_rot,fid105pos);
        tf::Quaternion quat110, quat105;

        //TODO
        tf::quaternionMsgToTF(fid105.transform.rotation, quat105);
        tf::quaternionMsgToTF(fid110.transform.rotation, quat110);
        //quat110 = q_rot*quat110;
        //quat105 = q_rot*quat105;
        //quatmid = quat105.slerp(quat110, 0.5);
        //q_rot2 = tf::Quaternion(-0.5,0.5,-0.5,-0.5);

        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "zed_left_camera";
        goal.target_pose.header.stamp = ros::Time::now();
        tf::Vector3 midvec;
        midvec = (fid105pos+fid110pos)/2;
        midvec = push_and_mean_list(queue_, midvec, ELEMS);
        if (queue_.size() < ELEMS) {
            return;
        }
        queue_.clear();
        tf::Quaternion q_rot2, q_new;
        double r=0, p=0, y=atan2((fid110pos.x()-fid105pos.x()), -(fid110pos.y()-fid105pos.y()));  // Rotate the previous pose by 180* about Z
        if((y<-1.57) or (y > 1.57))
        {
            y = y+3.14;
        }
        q_rot2 = tf::createQuaternionFromRPY(r, p, y);
        q_new = q_rot2;  // Calculate the new orientation
        q_new.normalize();
        geometry_msgs::Quaternion qMsg;
        tf::quaternionTFToMsg(q_new, qMsg);
        double _dist;
        if (flag_positioning_) {
            _dist = START_DIST;
            flag_positioning_= false;
        } else {
            _dist = FIN_DIST;
            flag_positioning_ = true;
        }

        goal.target_pose.pose.position.x = midvec.x() + cos(y) * _dist;
        goal.target_pose.pose.position.y = midvec.y() + sin(y) * _dist;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation = qMsg;

        ROS_INFO("Got coordinate: %f, %f, %f", goal.target_pose.pose.position.x, goal.target_pose.pose.position.y, goal.target_pose.pose.position.z);
        ROS_INFO("Got quaternion: x:%f, y:%f, z:%f,w: %f", goal.target_pose.pose.orientation.x, goal.target_pose.pose.orientation.y, goal.target_pose.pose.orientation.z, goal.target_pose.pose.orientation.w);
        ROS_INFO("Sending goal to move_base");

        ac.sendGoal(goal);
        ac.waitForResult();

        std::map<std::string, std::string> result;
        if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
            ROS_INFO("Malish, the robot moved to goal");
            if(flag_positioning_== false)
                result["reach"] = "pose_OK";
            else
                result["reach"] = "OK";
        } else {
            ROS_INFO("The robot failed to move to goal for some reason");
            result["err"] = "Failed";
            ros::Duration(1).sleep();
        }

        //TODO some results

        interface::ProcessInterface::Result alibResultOk;
        this->sendFeedback(result);

        rosLinkClientPtr_->setSucceeded(alibResultOk,"OK");
        ROS_INFO("Sended ResultOk");

        goal_active = false;
    }
}
void CargoFinder::sendFeedback(std::map<std::string, std::string> &result) {

    interface::ProcessInterface::Feedback feedback;
    core_msgs::DataPL dataPL;
    dataPL.command = "execute";
    dataPL.id = rosLinkClientPtr_->getId();
    dataPL.type = "feedback";
    core_msgs::KeyValue keyValue;

    for (auto it = result.begin(); it != result.end(); ++it) {
        keyValue.key = it->first;
        keyValue.value = it->second;

        ROS_INFO("Status: %s : %s", keyValue.key.c_str(), keyValue.value.c_str());

        dataPL.states.push_back(keyValue);
    }

    feedback.feedback.push_back(std::move(dataPL));
    rosLinkClientPtr_->publishFeedback(std::move(feedback));
}
