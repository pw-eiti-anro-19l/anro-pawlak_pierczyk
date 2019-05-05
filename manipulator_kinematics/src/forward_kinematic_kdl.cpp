#include <iostream>
#include <map>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "geometry_msgs/PoseStamped.h"

#include "kdl/frames.hpp"
#include "kdl/chain.hpp"
#include "kdl/segment.hpp"
#include "kdl/jntarray.hpp"
#include "kdl/chainfksolverpos_recursive.hpp"


void calculate_kinematic(const sensor_msgs::JointState&);

//publisher used to publish results to rviz
ros::Publisher pub;

//main chain of the manipulator
KDL::Chain kdlChain;


int main(int argc, char** argv){
  


    /**********************/
    /*   Initialization   */
    /**********************/

    ros::init(argc, argv, "forward_kinematic_kdl");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joint_states", 100, &calculate_kinematic);
    pub = node.advertise<geometry_msgs::PoseStamped>("forward_kinematic_kdl", 10);


    /**************************/
    /*   Load dh parameters   */
    /**************************/

    //map of the link's DH parameters
    std::map<std::string, std::vector<double>> links;

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



    /***********************************/
    /*   Initialization of the chain   */
    /***********************************/

    //Base link
    KDL::Frame frame = KDL::Frame::DH_Craig1989(links["i1"].at(0), links["i1"].at(1), 0, 0);
    KDL::Joint joint(KDL::Joint::None);
    kdlChain.addSegment(KDL::Segment(joint, frame));

    //Link 1
    frame = KDL::Frame::DH_Craig1989(links["i2"].at(0), links["i2"].at(1), links["i1"].at(2), links["i1"].at(3));
    joint = KDL::Joint(KDL::Joint::RotZ);
    kdlChain.addSegment(KDL::Segment(joint, frame));

    //Link 2
    frame = KDL::Frame::DH_Craig1989(links["i3"].at(0), links["i3"].at(1), links["i2"].at(2), links["i2"].at(3));
    joint = KDL::Joint(KDL::Joint::RotZ);
    kdlChain.addSegment(KDL::Segment(joint, frame));

    //Link 3
    frame = KDL::Frame::DH_Craig1989(0, 0, links["i3"].at(2), links["i3"].at(3));
    joint = KDL::Joint(KDL::Joint::RotZ);
    kdlChain.addSegment(KDL::Segment(joint, frame));

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
    KDL::JntArray jointAngles = KDL::JntArray(3);
    jointAngles(0) = msg.position.at(0);
    jointAngles(1) = msg.position.at(1);
    jointAngles(2) = msg.position.at(2);

    //position of the end in KDL::Frame object
    KDL::Frame end_position;

    //initial end position in KDL::Frame
    KDL::ChainFkSolverPos_recursive FKSolver(kdlChain);
    FKSolver.JntToCart(jointAngles, end_position);


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