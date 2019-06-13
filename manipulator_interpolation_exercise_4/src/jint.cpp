#include <math.h>
#include <vector>
#include "ros/ros.h"
#include "sensor_msgs/JointState.h"
#include "manipulator_interpolation/jint_control_srv.h"
#include "manipulator_interpolation/joint_state.h"
#include "nav_msgs/Path.h"

nav_msgs::Path path;
ros::Publisher path_pub;

bool interpolation(manipulator_interpolation::jint_control_srv::Request  &req,
                   manipulator_interpolation::jint_control_srv::Response &res)
{  
   
    /*******************************/
    /*  Check correctness of data  */
    /*******************************/

    if(req.initial_angles[1] < -M_PI || req.initial_angles[1] > 0 || req.initial_angles[2] < -M_PI_2 || req.initial_angles[1] > M_PI_2)
    {
        ROS_INFO("Incorrect initial angles values!");
        return false;
    }
    if(req.final_angles[1] < -M_PI || req.final_angles[1] > 0 || req.final_angles[2] < -M_PI_2 || req.final_angles[1] > M_PI_2)
    {
        ROS_INFO("Incorrect final angles values!");
        return false;
    }
    if(req.duration <= 0)
    {
        ROS_INFO("Incorrect time value!");
        return false;
    }
    if(req.interpolation_type.compare("linear") != 0 && req.interpolation_type.compare("cubic") != 0)
    {
        ROS_INFO("Incorrect interpolation type!");
        return false;
    }

    //note time stamps
    ros::Time begin = ros::Time::now();
    ros::Duration duration(req.duration);
    ros::Time end = begin + duration; 


    /*************************************/
    /*       Prepare topic & message     */
    /*************************************/

    ros::NodeHandle n;
    ros::Publisher interpolation = n.advertise<sensor_msgs::JointState>("joint_states", 1000);
    ros::ServiceClient client = n.serviceClient<manipulator_interpolation::joint_state>("forward_kinematic");

    //wait for robot_state_publisher to run
    while (interpolation.getNumSubscribers() < 1)
    {
        if (!ros::ok())
            return false;

        ROS_WARN_ONCE("Please run robot_state_publisher!");
        sleep(1);
    }

    //create JointState message
    sensor_msgs::JointState joints;

    joints.name = std::vector<std::string>{"baseb_to_link1", "link1_to_link2a", "link2b_to_link3a"};
    joints.position = std::vector<double>{req.initial_angles[0], req.initial_angles[1], req.initial_angles[2]};
    joints.velocity = std::vector<double>{0.0, 0.0, 0.0};

    /***************************************/
    /*       Position calculations         */
    /***************************************/

    ros::Rate loop_rate(60);
    path.poses.clear();

    /*****************************************/
    /********** Linear calculations **********/
    
    if(req.interpolation_type.compare("linear") == 0)
    {
      joints.velocity[0] = (req.final_angles[0] - req.initial_angles[0]) / duration.toSec();
      joints.velocity[1] = (req.final_angles[1] - req.initial_angles[1]) / duration.toSec();
      joints.velocity[2] = (req.final_angles[2] - req.initial_angles[2]) / duration.toSec();

      while (ros::Time::now() < end)
      {
        joints.position[0] = req.initial_angles[0] + ((req.final_angles[0] - req.initial_angles[0]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        joints.position[1] = req.initial_angles[1] + ((req.final_angles[1] - req.initial_angles[1]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        joints.position[2] = req.initial_angles[2] + ((req.final_angles[2] - req.initial_angles[2]) / duration.toSec())*(ros::Time::now().toSec() - begin.toSec());
        joints.header.stamp = ros::Time::now();

        interpolation.publish(joints);

        //get forward kinematic
        manipulator_interpolation::joint_state srv;
        srv.request.angles = joints;
        client.call(srv);

        //publish Path
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "link_baseb";
        path.poses.push_back(srv.response.pose);
        path_pub.publish(path);

        loop_rate.sleep();
      }
    }

    /*****************************************/
    /********** Trapeze calculations **********/

    else if(req.interpolation_type.compare("cubic") == 0)
    {
      // calculate a coefficients
      std::vector<double> a;
      a.push_back(req.initial_velocity[0]*(end.toSec() - begin.toSec()) - 
                 (req.final_angles[0] - req.initial_angles[0]));
      a.push_back(req.initial_velocity[1]*(end.toSec() - begin.toSec()) - 
                 (req.final_angles[1] - req.initial_angles[1]));
      a.push_back(req.initial_velocity[2]*(end.toSec() - begin.toSec()) - 
                 (req.final_angles[2] - req.initial_angles[2]));

      // calculate b coefficients
      std::vector<double> b;
      b.push_back(-req.final_velocity[0]*(end.toSec() - begin.toSec()) + 
                 (req.final_angles[0] - req.initial_angles[0]));
      b.push_back(-req.final_velocity[1]*(end.toSec() - begin.toSec()) + 
                 (req.final_angles[1] - req.initial_angles[1]));
      b.push_back(-req.final_velocity[2]*(end.toSec() - begin.toSec()) + 
                 (req.final_angles[2] - req.initial_angles[2]));

      while (ros::Time::now() < end)
      {
        double t = (ros::Time::now().toSec() - begin.toSec()) / (end.toSec() - begin.toSec());

        // calculate positions
        joints.position[0] = (1 - t) * req.initial_angles[0] + 
                             t * req.final_angles[0]         +
                             t * (1 - t) * (a[0] * (1 - t) + b[0] * t);

        joints.position[1] = (1 - t) * req.initial_angles[1] + 
                             t * req.final_angles[1]         +
                             t * (1 - t) * (a[1] * (1 - t) + b[1] * t);

        joints.position[2] = (1 - t) * req.initial_angles[2] + 
                             t * req.final_angles[2]         +
                             t * (1 - t) * (a[2] * (1 - t) + b[2] * t);


        if (joints.position[1] < -M_PI || joints.position[1] > 0 || joints.position[2] < -M_PI_2 || joints.position[1] > M_PI_2)
        {
          ROS_INFO("Calculated interpolation tries to introduce prohibited angles values!");
          return false;
        }

        // calculate velocities
        joints.velocity[0] = (req.final_angles[0] - req.initial_angles[0]) / (end.toSec() - begin.toSec()) +
                             (1 - 2*t) * (a[0] * (1 - t) + b[0]*t) / (end.toSec() - begin.toSec())              +
                             t * (1 - t) * (b[0] - a[0]) / (end.toSec() - begin.toSec());

        joints.velocity[1] = (req.final_angles[1] - req.initial_angles[1]) / (end.toSec() - begin.toSec()) +
                             (1 - 2*t) * (a[1] * (1 - t) + b[1]*t) / (end.toSec() - begin.toSec())              +
                             t * (1 - t) * (b[1] - a[1]) / (end.toSec() - begin.toSec());

        joints.velocity[2] = (req.final_angles[2] - req.initial_angles[2]) / (end.toSec() - begin.toSec()) +
                             (1 - 2*t) * (a[2] * (1 - t) + b[2]*t) / (end.toSec() - begin.toSec())              +
                             t * (1 - t) * (b[2] - a[2]) / (end.toSec() - begin.toSec());

        joints.header.stamp = ros::Time::now();

        interpolation.publish(joints);

        //get forward kinematic
        manipulator_interpolation::joint_state srv;
        srv.request.angles = joints;
        client.call(srv);

        //publish Path
        path.header.stamp = ros::Time::now();
        path.header.frame_id = "link_baseb";
        path.poses.push_back(srv.response.pose);
        path_pub.publish(path);

        loop_rate.sleep();
      }
    }

    /***************************************/
    /*          Final position              */
    /***************************************/

    joints.header.stamp = ros::Time::now();
    joints.position = std::vector<double> {req.final_angles[0], req.final_angles[1], req.final_angles[2]};
    joints.velocity = std::vector<double> {0.0, 0.0, 0.0};

    interpolation.publish(joints);

    return true;
}





int main(int argc, char **argv)
{
  ros::init(argc, argv, "jint");
  ros::NodeHandle n;
  ros::ServiceServer interpolation_service = n.advertiseService("interpolation", interpolation);
  path_pub = n.advertise<nav_msgs::Path>("manipulator_path", 1000);
  ros::spin();
  return 0;
}