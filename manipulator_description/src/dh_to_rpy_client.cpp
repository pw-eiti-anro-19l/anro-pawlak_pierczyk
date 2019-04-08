#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "geometry_msgs/Vector3.h"
#include "manipulator_description/DhToRPY.h"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "client");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<manipulator_description::DhToRPY>("dh_to_rpy");
  manipulator_description::DhToRPY srv;
  srv.request.linknum = 3;
  srv.request.ai_1 = {0, 0, 0};
  srv.request.di = {0, 0, 0};
  srv.request.alhi_1 = {0, -1.570796, 0};
  srv.request.fii = {0, 0, 0};

  client.call(srv);

  return 0;
}