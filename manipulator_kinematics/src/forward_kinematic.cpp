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

    ros::init(argc, argv, "forward_kinematic");
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
    /*  Public initial position   */   
    /******************************/

    /*<----------------------------------------------------------------->*/
    /*<-- BUG - when initial tf post is ommited the node won't        -->*/
    /*<-- publish any messages to /tf topic what results in lack of   -->*/
    /*<-- desired 'forward_kinematic' frame in the rviz.              -->*/
    /*<----------------------------------------------------------------->*/

    tf::TransformBroadcaster tf_broad;
    tf::Transform end;

    tf_broad.sendTransform(tf::StampedTransform(end, ros::Time::now(), "link_baseb", "forward_kinematic"));



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

    //tf's tools required to publish end position to the /tf topic
    tf::TransformBroadcaster tf_broad;
    tf::Transform end;

    //transform KDL::Frame position to tf::Transform
    end.setOrigin(tf::Vector3(end_position.p.x(), end_position.p.y(), end_position.p.z()));

    //transform KDL::Frame rotation to tf::Transform
    double x, y, z, w;
    end_position.M.GetQuaternion(x, y, z, w);
    end.setRotation(tf::Quaternion(x, y, z, w));

    //send end position to /tf topic
    tf_broad.sendTransform(tf::StampedTransform(end, ros::Time::now(), "link_baseb", "forward_kinematic"));

}