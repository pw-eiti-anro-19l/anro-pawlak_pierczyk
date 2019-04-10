#include <iostream>
#include "ros/ros.h"
#include "kdl/frames.hpp"
#include "geometry_msgs/Vector3.h"
#include "manipulator_description/DhToRPY.h"

using namespace std;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "getrpy");
  ros::NodeHandle n;

  ros::ServiceClient client = n.serviceClient<manipulator_description::DhToRPY>("dhtorpy");
  manipulator_description::DhToRPY srv;

  if(n.hasParam("ai_1"))
            n.getParam("ai_1", srv.request.ai_1);
  if(n.hasParam("alhi_1"))
            n.getParam("alhi_1", srv.request.alhi_1);
  if(n.hasParam("di"))
            n.getParam("di", srv.request.di);
  if(n.hasParam("fii"))
            n.getParam("fii", srv.request.fii);
  if(n.hasParam("linknum"))
            n.param("linknum", (int&)srv.request.linknum, (int)srv.request.ai_1.size());

  client.call(srv);
  
  for(int i = 0; i < srv.response.roll.size(); i++){
      std::cout << setw(5) << left;
      cout << srv.response.roll[i] << " ";
  }
  cout << endl;
  for(int i = 0; i < srv.response.pitch.size(); i++){
      std::cout << setw(5) << left;
      cout << srv.response.pitch[i] << " ";
  }
  cout << endl;
  for(int i = 0; i < srv.response.roll.size(); i++){
      std::cout << setw(5) << left;
      cout << srv.response.yaw[i] << " ";
  }
  cout << endl;

  return 0;
}