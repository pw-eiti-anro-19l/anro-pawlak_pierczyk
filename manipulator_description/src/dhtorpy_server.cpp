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

    Rotation rot = frame.M;
    Vector position = frame.p;

    rot.GetRPY(roll, pitch, yaw);

    res.roll.push_back(roll);
    res.pitch.push_back(pitch);
    res.yaw.push_back(yaw);
    
    res.x.push_back(position.x());
    res.y.push_back(position.y());
    res.z.push_back(position.z());
  }

  return true;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dhtorpy_server");
  ros::NodeHandle n;

  ros::ServiceServer service = n.advertiseService("dhtorpy", dhtorpy);
  ros::spin();

  return 0;
}
