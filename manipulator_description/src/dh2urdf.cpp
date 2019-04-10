#include <iostream>
#include <fstream>
#include <map>
#include <vector>
#include "ros/ros.h"
#include "kdl/frames.hpp"

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dhtorpy_server");
  ros::NodeHandle n;

  if (argc != 2)
  {
    ROS_INFO("Point path to the 'manipulator_description' folder!");
    return 1;
  }
  std::string path(argv[1]);

  //map of links dh parameters
  std::map<std::string, std::vector<double>> links;
  for(int i = 1; i <= 3; i++){
    //vector of the link dh parameters
    std::vector<double> link;
    if(!n.getParam("i" + std::to_string(i), link)){
      std::string INFO = "No i"  + std::to_string(i) + "link on the server.";
      ROS_INFO(INFO.c_str());
      return 1;
    }
    //add link to map
    links["i" + std::to_string(i)] = link;
  }

  //open file with ardf params
  std::ofstream urdf;
  urdf.open(path + "/param/urdfparam.yaml");


  std::vector<double> length;
  std::vector<double> rpy(3);

  //calculate joints position and orientation and print to cout
  for(int i = 1; i <= 3; i++)
  {
    //get joint's matrix
    KDL::Frame RotX = KDL::Frame(KDL::Rotation::RotX(links["i" + std::to_string(i)].at(1)));
    KDL::Frame TranX = KDL::Frame(KDL::Vector(links["i" + std::to_string(i)].at(0), 0, 0));
    KDL::Frame RotZ = KDL::Frame(KDL::Rotation::RotZ(links["i" + std::to_string(i)].at(3)));
    KDL::Frame TranZ = KDL::Frame(KDL::Vector(links["i" + std::to_string(i)].at(2), 0, 0));
    KDL::Frame frame = RotX * TranX * RotZ * TranZ;

    //get rotation in RPY
    KDL::Rotation rot = frame.M;
    rot.GetRPY(rpy.at(0), rpy.at(1), rpy.at(2));
    //get position
    KDL::Vector pos = frame.p;

    //write to file .yaml
    if(i > 1)
      urdf << "  len: " << links["i" + std::to_string(i)].at(0) << std::endl;  
    urdf << "i" + std::to_string(i) + ":" << std::endl;
    urdf << "  rpy: " << rpy.at(0) << " "
                      << rpy.at(1) << " "
                      << rpy.at(2) << std::endl;
    urdf << "  xyz: " << pos.x() << " "
                      << pos.y() << " "
                      << pos.z() << std::endl;
    if(i == 3)
      urdf << "  len: " << 0 << std::endl;

  }
  urdf.close();

  return 0;
}
