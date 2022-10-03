#ifndef CODE_HPP
#define CODE_HPP

#include <vector>
#include <geometry_msgs/Point.h>

namespace qr_tracking
{
  class Code
  {
    public:
    Code(){}
    ~Code(){}

    bool operator==(const Code& rhs)
    {
      double dx = x - rhs.x;
      double dy = y - rhs.y;
      double dz = z - rhs.z;

      double dl_sqr = dx*dx + dy*dy + dz*dz;

      return ((dl_sqr < 10.0) ? true : false);
    }

    bool operator!=(const Code& rhs)
    {
      double dx = x - rhs.x;
      double dy = y - rhs.y;
      double dz = z - rhs.z;

      double dl_sqr = dx*dx + dy*dy + dz*dz;

      return ((dl_sqr > 10.0) ? false : true);
    }

    public:
    /*Pixel coordinates*/
    int cx, cy;
    std::vector<geometry_msgs::Point> points;
    
    /*3D world coordinates*/
    double x, y, z;

    std::string type;
    std::string data;
  };
}

#endif