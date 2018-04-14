#ifndef OPTIMAL_PLANNER_H
#define OPTIMAL_PLANNER_H

#include "planner.h"

namespace discrete_planner
{
  class OptimalPlanner : public Planner
  {
  public:
    OptimalPlanner()
    {
      type_ = "OptimalPlanner";
    }
    ~OptimalPlanner() {}

    std::deque<Pose> search(const std::vector<std::vector<bool>>& world_state,
                            const Pose& robot_pose, const Pose& goal_pose);
  };
}

#endif // OPTIMAL_PLANNER_H
