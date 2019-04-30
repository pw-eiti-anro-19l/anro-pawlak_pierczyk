#include <iostream>
#include <map>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "kdl/frames.hpp"
#include "sensor_msgs/JointState.h"

void calculate_kinematic(const sensor_msgs::JointState&);

KDL::Frame end_position;

int main(int argc, char** argv){
  
    /**********************/
    /*   Initialization   */
    /**********************/

    ros::init(argc, argv, "simple_kinematic");
    ros::NodeHandle node;

    //ros::Subscriber sub = node.subscribe("JointState", 1000, calculate_kinematic);

    tf::TransformBroadcaster tf_broad;
    tf::Transform end;

    /**************************/
    /*   Load dh parameters   */
    /**************************/

    //map of links dh parameters
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

    /******************************/
    /* Calculate initial position */
    /******************************/

    //vector of frames representing link's position in previous link's frame of reference
    std::vector<KDL::Frame> reference_frames;

    for(int i = 1; i <= 3; i++)
    {
        //get link's refference frame
        KDL::Frame RotX = KDL::Frame(KDL::Rotation::RotX(links["i" + std::to_string(i)].at(1)));
        KDL::Frame TranX = KDL::Frame(KDL::Vector(links["i" + std::to_string(i)].at(0), 0, 0));
        KDL::Frame RotZ = KDL::Frame(KDL::Rotation::RotZ(links["i" + std::to_string(i)].at(3)));
        KDL::Frame TranZ = KDL::Frame(KDL::Vector(links["i" + std::to_string(i)].at(2), 0, 0));
        KDL::Frame frame = RotX * TranX * RotZ * TranZ;

        reference_frames.push_back(frame); 
   
    }

    //initial end position in KDL::Frame
    end_position = reference_frames.at(0)*reference_frames.at(1)*reference_frames.at(2);

    double x, y, z, w;
    end_position.M.GetQuaternion(x, y, z, w);

    //transform KDL::Frame position to tf::Transform
    end.setOrigin(tf::Vector3(end_position.p.x(), end_position.p.y(), end_position.p.z()));
    end.setRotation(tf::Quaternion(x, y, z, w));

    /*****************/
    /*   Main loop   */
    /*****************/

    ros::Rate rate(10.0);
    while (node.ok())
    {
        tf_broad.sendTransform(tf::StampedTransform(end, ros::Time::now(), "link_baseb", "simple_kinematic"));
        rate.sleep();
    }
    return 0;
}



void calculate_kinematic(const sensor_msgs::JointState& msg){

}