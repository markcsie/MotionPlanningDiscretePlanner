#include "optimal_planner.h"

#include "util.h"

#include <iostream>
#include <unordered_map>

namespace discrete_planner
{
  std::deque<Pose> OptimalPlanner::search(const std::vector<std::vector<bool>>& world_state,
                                          const Pose& start_pose, const Pose& goal_pose)
  {
    if (world_state[start_pose.x][start_pose.y] || world_state[goal_pose.x][goal_pose.y])
    {
      std::cout << "incorrect inputs" << std::endl;
      return {};
    }

    int x_size = world_state.size();
    int y_size = world_state[0].size();

    // BFS
    std::vector<std::vector<bool>> visited_map(x_size, std::vector<bool>(y_size, false));
    std::queue<Pose> bfs_queue;
    bfs_queue.push(start_pose);
    std::unordered_map<int, Pose> parent_map;
    while (!bfs_queue.empty())
    {
      Pose pose = bfs_queue.front();
      visited_map[pose.x][pose.y] = true;
      if (pose == goal_pose)
      {
        // backtrace
        std::deque<Pose> path;
        while (pose != start_pose)
        {
          path.push_front(pose);
          pose = parent_map[pose.x * y_size + pose.y];
        }
        path.push_front(start_pose);
        return path;
      }

      for (const Pose& action : GetValidActions(world_state, pose))
      {
        int next_x = pose.x + action.x;
        int next_y = pose.y + action.y;
        if (!visited_map[next_x][next_y])
        {
          bfs_queue.push(Pose(next_x, next_y));
          parent_map[next_x * y_size + next_y] = pose;
        }
      }

      bfs_queue.pop();
    }

    return {};
  }
}
