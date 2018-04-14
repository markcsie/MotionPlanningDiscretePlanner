#ifndef RANDOM_PLANNER_H
#define RANDOM_PLANNER_H

#include <cmath>

#include "planner.h"

namespace discrete_planner
{
  class RandomPlanner : public Planner
  {
  public:
    RandomPlanner(const unsigned int &max_step_number,
                  const unsigned int &seed) : max_step_number_(max_step_number),
                                              memory_size_(sqrt(max_step_number)),
                                              seed_(seed)
    {
      type_ = "RandomPlanner";
      srand(seed_);
    }
    ~RandomPlanner() {}

    std::deque<Pose> search(const std::vector<std::vector<bool>>& world_state,
                            const Pose& robot_pose, const Pose& goal_pose);
  protected:
    unsigned int max_step_number_;
    unsigned int memory_size_;
    unsigned int seed_;
  };
}

#endif // RANDOM_PLANNER_H
