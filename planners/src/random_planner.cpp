#include "random_planner.h"

#include "util.h"

#include <iostream>

namespace discrete_planner
{
  std::deque<Pose> RandomPlanner::search(const std::vector<std::vector<bool>>& world_state,
                                         const Pose& start_pose, const Pose& goal_pose)
  {
    if (world_state[start_pose.x][start_pose.y] || world_state[goal_pose.x][goal_pose.y])
    {
      std::cout << "incorrect inputs" << std::endl;
      return {};
    }

    int x_size = world_state.size();
    int y_size = world_state[0].size();

    std::vector<std::vector<bool>> visited_map(x_size, std::vector<bool>(y_size, false));
    std::queue<Pose> memory({start_pose});
    visited_map[start_pose.x][start_pose.y] = true;

    std::vector<Pose> valid_actions = GetValidActions(world_state, start_pose);
    std::deque<Pose> path({start_pose});
    unsigned int steps = 0;
    Pose pose = start_pose;
    while (steps < max_step_number_)
    {
      std::vector<Pose>::const_iterator it(valid_actions.begin());
      std::advance(it, rand() % valid_actions.size());
      int next_x = pose.x + (*it).x;
      int next_y = pose.y + (*it).y;
      if (!visited_map[next_x][next_y] || valid_actions.size() == 1)
      {
        pose.x = next_x;
        pose.y = next_y;
        valid_actions = GetValidActions(world_state, pose);
      }
      else
      {
        valid_actions.erase(it);
        continue;
      }

      path.push_back(pose);
      memory.push(pose);
      visited_map[pose.x][pose.y] = true;
      if (memory.size() > memory_size_)
      {
        visited_map[memory.front().x][memory.front().y] = false;
        memory.pop();
      }
      ++steps;
      if (pose == goal_pose)
      {
        return path;
      }
    }
    return {};
  }
}
