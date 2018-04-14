#ifndef POSE_H
#define POSE_H

namespace discrete_planner
{
  struct Pose
  {
    int x;
    int y;
    Pose() : x(0), y(0) {}
    Pose(const int& x, const int& y) : x(x), y(y) {}
    inline bool operator==(const Pose& rhs) const
    {
      return (x == rhs.x && y == rhs.y);
    }
    inline bool operator!=(const Pose& rhs) const
    {
      return !(*this == rhs);
    }
  };
}

#endif // POSE_H
