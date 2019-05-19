#include <math.h>
#include <vector>

#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "manipulator_interpolation/oint_control_srv.h"
#include "manipulator_interpolation/joint_state.h"
#include "nav_msgs/Path.h"
#include "geometry_msgs/PoseStamped.h"
#include "kdl/frames.hpp"

nav_msgs::Path path;
ros::Publisher pub;
ros::Publisher path_pub;


bool interpolation(manipulator_interpolation::oint_control_srv::Request  &req,
                   manipulator_interpolation::oint_control_srv::Response &res)
{  

    //note time stamps
    ros::Time begin = ros::Time::now();
    ros::Duration duration(req.duration);
    ros::Time end = begin + duration; 


    /*************************************/
    /*     Prepare message & message     */
    /*************************************/

    //create PoseStamped message
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "link_basea";

    /***************************************/
    /*       Position calculations         */
    /***************************************/

    ros::Rate loop_rate(60);
    path.poses.clear();

    /*****************************************/
    /********** Linear calculations **********/
    
    if(req.interpolation_type.compare("linear") == 0)
    {
      while (ros::Time::now() < end)
      {
        pose.pose.position.x = req.pose.initial_position[0] + ((req.pose.final_position[0] - req.pose.initial_position[0]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        pose.pose.position.y = req.pose.initial_position[1] + ((req.pose.final_position[1] - req.pose.initial_position[1]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        pose.pose.position.z = req.pose.initial_position[2] + ((req.pose.final_position[2] - req.pose.initial_position[2]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());

        double R = req.pose.initial_orientation[0] + ((req.pose.final_orientation[0] - req.pose.initial_orientation[0]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        double P = req.pose.initial_orientation[1] + ((req.pose.final_orientation[1] - req.pose.initial_orientation[1]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        double Y = req.pose.initial_orientation[2] + ((req.pose.final_orientation[2] - req.pose.initial_orientation[2]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        
        KDL::Rotation rotation = KDL::Rotation::RPY(R, P, Y);

        rotation.GetQuaternion(pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z,
                               pose.pose.orientation.w);

        pose.header.stamp = ros::Time::now();
        pub.publish(pose);

        //publish Path
        path.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);

        loop_rate.sleep();
      }
    }

    /*****************************************/
    /********** Trapeze calculations **********/

    else if(req.interpolation_type.compare("cubic") == 0)
    {
      // calculate a coefficients for position
      std::vector<double> a;
      a.push_back(req.vel.initial_velocity[0]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_position[0] - req.pose.initial_position[0]));
      a.push_back(req.vel.initial_velocity[1]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_position[1] - req.pose.initial_position[1]));
      a.push_back(req.vel.initial_velocity[2]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_position[2] - req.pose.initial_position[2]));

      // calculate b coefficients for position
      std::vector<double> b;
      b.push_back(-req.vel.final_velocity[0]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_position[0] - req.pose.initial_position[0]));
      b.push_back(-req.vel.final_velocity[1]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_position[1] - req.pose.initial_position[1]));
      b.push_back(-req.vel.final_velocity[2]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_position[2] - req.pose.initial_position[2]));

      // calculate a coefficients for orientation
      std::vector<double> a_ang;
      a_ang.push_back(req.vel.initial_ang_velocity[0]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_orientation[0] - req.pose.initial_orientation[0]));
      a_ang.push_back(req.vel.initial_ang_velocity[1]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_orientation[1] - req.pose.initial_orientation[1]));
      a_ang.push_back(req.vel.initial_ang_velocity[2]*(end.toSec() - begin.toSec()) - 
                 (req.pose.final_orientation[2] - req.pose.initial_orientation[2]));

      // calculate b coefficients for orientation
      std::vector<double> b_ang;
      b_ang.push_back(-req.vel.final_ang_velocity[0]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_orientation[0] - req.pose.initial_orientation[0]));
      b_ang.push_back(-req.vel.final_ang_velocity[1]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_orientation[1] - req.pose.initial_orientation[1]));
      b_ang.push_back(-req.vel.final_ang_velocity[2]*(end.toSec() - begin.toSec()) + 
                 (req.pose.final_orientation[2] - req.pose.initial_orientation[2]));

      while (ros::Time::now() < end)
      {
        double t = (ros::Time::now().toSec() - begin.toSec()) / (end.toSec() - begin.toSec());

        pose.pose.position.x = (1 - t) * req.pose.initial_position[0] + t * req.pose.final_position[0] + t * (1 - t) * (a[0] * (1 - t) + b[0]*t);
        pose.pose.position.y = (1 - t) * req.pose.initial_position[1] + t * req.pose.final_position[1] + t * (1 - t) * (a[1] * (1 - t) + b[1]*t);
        pose.pose.position.z = (1 - t) * req.pose.initial_position[2] + t * req.pose.final_position[2] + t * (1 - t) * (a[2] * (1 - t) + b[2]*t);

        double R = (1 - t) * req.pose.initial_orientation[0] + t * req.pose.final_orientation[0] + t * (1 - t) * (a_ang[0] * (1 - t) + b_ang[0]*t);
        double P = (1 - t) * req.pose.initial_orientation[1] + t * req.pose.final_orientation[1] + t * (1 - t) * (a_ang[1] * (1 - t) + b_ang[0]*t);
        double Y = (1 - t) * req.pose.initial_orientation[2] + t * req.pose.final_orientation[2] + t * (1 - t) * (a_ang[2] * (1 - t) + b_ang[0]*t);

        KDL::Rotation rotation = KDL::Rotation::RPY(R, P, Y);

        rotation.GetQuaternion(pose.pose.orientation.x,
                               pose.pose.orientation.y,
                               pose.pose.orientation.z,
                               pose.pose.orientation.w);

        pose.header.stamp = ros::Time::now();
        pub.publish(pose);

        //publish Path
        path.header.stamp = ros::Time::now();
        path.poses.push_back(pose);
        path_pub.publish(path);

        loop_rate.sleep();
      }
    }

    /***************************************/
    /*          Final position              */
    /***************************************/

    pose.pose.position.x = req.pose.final_position[0];
    pose.pose.position.y = req.pose.final_position[1];
    pose.pose.position.z = req.pose.final_position[2];

    double R = req.pose.final_orientation[0];
    double P = req.pose.final_orientation[1];
    double Y = req.pose.final_orientation[2];

    KDL::Rotation rotation = KDL::Rotation::RPY(R, P, Y);

    rotation.GetQuaternion(pose.pose.orientation.x,
                           pose.pose.orientation.y,
                           pose.pose.orientation.z,
                           pose.pose.orientation.w);

    pose.header.stamp = ros::Time::now();
    pub.publish(pose);

    return true;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "oint");
  ros::NodeHandle n;
  ros::ServiceServer o_interpolation_service = n.advertiseService("o_interpolation", interpolation);
  
  pub = n.advertise<geometry_msgs::PoseStamped>("o_frame", 10);
  path_pub = n.advertise<nav_msgs::Path>("frame_path", 1000);

  path.header.frame_id = "link_basea";
  
  ros::spin();
  return 0;
}