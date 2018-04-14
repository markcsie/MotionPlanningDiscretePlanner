#ifndef UTIL_H
#define UTIL_H

#include <vector>

#include "pose.h"

static std::vector<Pose> GetValidActions(const std::vector<std::vector<bool>>& world_state,
                                         const Pose& robot_pose)
{
  int x_size = world_state.size();
  int y_size = world_state[0].size();
  std::vector<Pose> actions;
  if (robot_pose.x - 1 >= 0 && !world_state[robot_pose.x - 1][robot_pose.y])
  {
    actions.push_back(Pose(-1, 0));
  }
  if (robot_pose.x + 1 < x_size && !world_state[robot_pose.x + 1][robot_pose.y])
  {
    actions.push_back(Pose(1, 0));
  }
  if (robot_pose.y - 1 >= 0 && !world_state[robot_pose.x][robot_pose.y - 1])
  {
    actions.push_back(Pose(0, -1));
  }
  if (robot_pose.y + 1 < y_size && !world_state[robot_pose.x][robot_pose.y + 1])
  {
    actions.push_back(Pose(0, 1));
  }

  return actions;
}

#endif // UTIL_H
