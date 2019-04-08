#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "geometry_msgs/Vector3.h"
#include "manipulator_description/DhToRPY.h"


using namespace KDL;

bool dhtorpy(manipulator_description::DhToRPY::Request  &req,
         manipulator_description::DhToRPY::Response &res)
{
  double roll;
  double pitch;
  double yaw;

  for(int i = 0; i < req.linknum; i++)
  {
    Frame frame = Frame::DH(req.ai_1[i], req.alhi_1[i], req.di[i], req.fii[i]);
    frame = frame.Inverse();

    Rotation rot = frame.M;
    Vector position = frame.p;

    rot.GetRPY(roll, pitch, yaw);

    res.roll.push_back(roll);
    res.pitch.push_back(pitch);
    res.yaw.push_back(yaw);
    
    std::cout << res.roll[i] << " ";
    std::cout << res.pitch[i] << " ";
    std::cout << res.yaw[i] << std::endl;

    geometry_msgs::Point point;
    point.x = position.x();
    point.y = position.y();
    point.z = position.z();
    res.position.push_back(point);
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dh_to_rpy_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("dh_to_rpy", dhtorpy);
  ROS_INFO("Ready to add two ints.");
  ros::spin();

  return 0;
}
