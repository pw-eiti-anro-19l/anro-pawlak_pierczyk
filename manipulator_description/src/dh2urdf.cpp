#include <iostream>
#include <map>
#include <vector>
#include "ros/ros.h"
#include "kdl/frames.hpp"

using namespace KDL;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "dhtorpy_server");
  ros::NodeHandle n;

  const int fieldsize = 5; //width of the print field
  int linknum; //number of links
  std::vector<double> rpy(3); //result rpy vector
  std::vector<double> xyz(3); //result translation vector

  //get links number
  if(n.hasParam("linknum")){
    n.getParam("linknum", linknum);
  }
  else linknum = 0;

  //map of links dh parameters
  std::map<std::string, std::vector<double>> links;
  for(int i = 1; i <= linknum; i++){
    //vector of the link dh parameters
    std::vector<double> link;
    if(n.hasParam("i" + std::to_string(i))){
      n.getParam("i" + std::to_string(i), link);
    }
    else{
      link.insert(link.cend(), {0, 0, 0, 0});
    }
    
    //add link to map
    links["i" + std::to_string(i)] = link;
  }

  //calculate joints position and orientation and print to cout
  std::cout << std::endl << std::endl << std::endl;
  for(int i = 1; i <= linknum; i++)
  {
    //get joint's matrix
    Frame frame = Frame::DH(links["i" + std::to_string(i)].at(0),
                            links["i" + std::to_string(i)].at(1),
                            links["i" + std::to_string(i)].at(2),
                            links["i" + std::to_string(i)].at(3));

    //get rotation in RPY
    Rotation rot = frame.M;
    //get position
    Vector pos = frame.p;

    //save RPY and pos to result vectors
    rot.GetRPY(rpy.at(0), rpy.at(1), rpy.at(2));
    pos.x(xyz.at(0));
    pos.y(xyz.at(1));
    pos.z(xyz.at(2));


    //print results
    std::cout.precision(3);
    std::cout << "joint " + std::to_string(i) + ":" << std::endl;
    std::cout << "   -rpy: " << std::setw(fieldsize) << rpy.at(0)
                                  << std::setw(fieldsize) << rpy.at(1)
                                  << std::setw(fieldsize) << rpy.at(2)
                                  << std::endl;
    std::cout << "   -xyz: " << std::setw(fieldsize) << xyz.at(0)
                                  << std::setw(fieldsize) << xyz.at(1)
                                  << std::setw(fieldsize) << xyz.at(2)
                                  << std::endl;
    std::cout << std::endl;
  }
  std::cout << std::endl << std::endl << std::endl;

  return 0;
}
