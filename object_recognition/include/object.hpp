#ifndef OBJECT_HPP_
#define OBJECT_HPP_

#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Vector3.h>

namespace object_recognition
{
  class Object
  {
    public:
    Object()  {}
    ~Object() {}

    public:
    geometry_msgs::Point vertices[8];
    tf2::Vector3 normals[4];

    double cx, cy, cz;
    std::string name;
  };
}

#endif