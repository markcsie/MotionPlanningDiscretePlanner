#ifndef PLANNER_H
#define PLANNER_H

#include <vector>
#include <queue>
#include <string>

#include "pose.h"

namespace discrete_planner
{
  class Planner
  {
  public:
    virtual std::deque<Pose> search(const std::vector<std::vector<bool>>& world_state,
                                    const Pose& robot_pose, const Pose& goal_pose) = 0;
    virtual ~Planner()
    {
    }

    const std::string &getType() const
    {
      return type_;
    }

  protected:
    std::string type_;
  };
}

#endif // PLANNER_H
