#include <iostream>
#include <map>
#include <vector>
#include "ros/ros.h"
#include "tf/transform_broadcaster.h"
#include "kdl/frames.hpp"
#include "sensor_msgs/JointState.h"



void calculate_kinematic(const sensor_msgs::JointState&);



//map of links dh parameters
std::map<std::string, std::vector<double>> links;
//position of the end in KDL::Frame object
KDL::Frame end_position;



int main(int argc, char** argv){
  


    /**********************/
    /*   Initialization   */
    /**********************/

    ros::init(argc, argv, "forward_kinematic_kdl");
    ros::NodeHandle node;

    ros::Subscriber sub = node.subscribe("joint_states", 100, &calculate_kinematic);



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



    /******************************/
    /* Calculate initial position */   
    /******************************/

    /*<----------------------------------------------------------------->*/
    /*<-- BUG - when initial calculations are ommited the node won't  -->*/
    /*<-- publish any messages to /tf topic what results in lack of    -->*/
    /*<-- desired 'forward_kinematic' frame in the rviz.               -->*/
    /*<----------------------------------------------------------------->*/

    tf::TransformBroadcaster tf_broad;
    tf::Transform end;

    /*<--  Here calculate forward kinematic with orocos' KDL -->*/
    /*<--  Place result in the end_position frame          -->*/
    
    //initial end position in KDL::Frame
    //end_position = 
    
    //transform KDL::Frame position to tf::Transform
    end.setOrigin(tf::Vector3(end_position.p.x(), end_position.p.y(), end_position.p.z()));

    //transform KDL::Frame rotation to tf::Transform
    double x, y, z, w;
    end_position.M.GetQuaternion(x, y, z, w);
    end.setRotation(tf::Quaternion(x, y, z, w));

    tf_broad.sendTransform(tf::StampedTransform(end, ros::Time::now(), "link_baseb", "forward_kinematic_kdl"));



    /************/
    /*   Spin   */
    /************/

    ros::spin();
    return 0;
}



void calculate_kinematic(const sensor_msgs::JointState& msg){

    tf::TransformBroadcaster tf_broad;
    tf::Transform end;

    //get positions of the link
    links["i1"].at(3) = msg.position.at(0);
    links["i2"].at(3) = msg.position.at(1);
    links["i3"].at(3) = msg.position.at(2);

    /*<--  Here calculate forward kinematic with orocos' KDL -->*/
    /*<--  Place result in the end_position frame           -->*/
    
    //initial end position in KDL::Frame
    //end_position = 
    
    //transform KDL::Frame position to tf::Transform
    end.setOrigin(tf::Vector3(end_position.p.x(), end_position.p.y(), end_position.p.z()));

    //transform KDL::Frame rotation to tf::Transform
    double x, y, z, w;
    end_position.M.GetQuaternion(x, y, z, w);
    end.setRotation(tf::Quaternion(x, y, z, w));

    tf_broad.sendTransform(tf::StampedTransform(end, ros::Time::now(), "link_baseb", "forward_kinematic_kdl"));
}