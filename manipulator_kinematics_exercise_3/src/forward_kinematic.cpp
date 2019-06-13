#include <iostream>
#include <map>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "kdl/frames.hpp"
#include "sensor_msgs/JointState.h"

void calculate_kinematic(const sensor_msgs::JointState&);

//publisher used to publish results to rviz
ros::Publisher pub;

//map of links dh parameters
std::map<std::string, std::vector<double>> links;
//position of the end in KDL::Frame object
KDL::Frame end_position;

int main(int argc, char** argv){
  


    /**********************/
    /*   Initialization   */
    /**********************/

    ros::init(argc, argv, "forward_kinematic");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joint_states", 100, &calculate_kinematic);
    pub = node.advertise<geometry_msgs::PoseStamped>("forward_kinematic", 10);



    /**************************/
    /*   Load dh parameters   */
    /**************************/

    for (int i = 1; i <= 3; i++)
    {

        //vector of the link dh parameters
        std::vector<double> link;

        if (!node.getParam("i" + std::to_string(i), link))
        {
            std::string INFO = "No i" + std::to_string(i) + "link on the server.";
            ROS_INFO(INFO.c_str());
            return 1;
        }

        //add link to map
        links["i" + std::to_string(i)] = link;
    }

    /************/
    /*   Spin   */
    /************/

    ros::spin();
    return 0;
}



void calculate_kinematic(const sensor_msgs::JointState& msg){

    /*************************/
    /*  Calculate position   */   
    /*************************/

    //get positions of the link
    links["i1"].at(3) = msg.position.at(0);
    links["i2"].at(3) = msg.position.at(1);
    links["i3"].at(3) = msg.position.at(2);

    //vector of frames representing link's position in previous link's frame of reference
    std::vector<KDL::Frame> reference_frames;

    for(int i = 1; i <= 3; i++)
    {   
        //get link's reference frame
        KDL::Frame frame = KDL::Frame::DH_Craig1989(links["i" + std::to_string(i)].at(0),
                                                    links["i" + std::to_string(i)].at(1),
                                                    links["i" + std::to_string(i)].at(2),
                                                    links["i" + std::to_string(i)].at(3));

        //store reference frame
        reference_frames.push_back(frame);
    }

    //initial end position in KDL::Frame
    end_position = reference_frames.at(0)*reference_frames.at(1)*reference_frames.at(2);
    
    
    /***********************/
    /*  Public  position   */   
    /***********************/

    geometry_msgs::PoseStamped pose;

    //basic info about the frame
    pose.header.stamp = ros::Time::now();
    pose.header.frame_id = "link_baseb";

    //position
    pose.pose.position.x = end_position.p.x();
    pose.pose.position.y = end_position.p.y();
    pose.pose.position.z = end_position.p.z();

    //get quaternion
    double x, y, z, w;
    end_position.M.GetQuaternion(x, y, z, w);
    //orientation
    pose.pose.orientation.x = x;
    pose.pose.orientation.y = y;
    pose.pose.orientation.z = z;
    pose.pose.orientation.w = w;


    while (pub.getNumSubscribers() < 1)
    {
        if (!ros::ok())
        {
            return;
        }

        ROS_WARN_ONCE("Please create a subscriber /forward_kinematic_kdl!");
        sleep(1);
    }

    pub.publish(pose);
}