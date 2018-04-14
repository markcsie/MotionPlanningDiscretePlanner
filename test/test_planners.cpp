#include <iostream>
#include <cassert>
#include <ctime>
#include <memory>

#include "planner.h"
#include "random_planner.h"
#include "optimal_planner.h"

std::deque<Pose> testPlanner(const std::shared_ptr<discrete_planner::Planner> &planner,
                             const std::vector<std::vector<bool>>& world_state,
                             const Pose& start_pose, const Pose& goal_pose,
                             const bool& has_solution)
{
  std::cout << "Test " << planner->getType() << std::endl;
  std::deque<Pose> path = planner->search(world_state, start_pose, goal_pose);
  if (!has_solution)
  {
    assert(path.empty());
    std::cout << "empty path" << std::endl;
    return path;
  }

  assert(path.front() == start_pose);
  assert(path.back() == goal_pose);
  for (const Pose& p : path)
  {
    assert(!world_state[p.x][p.y]);
    std::cout << "(" << p.x << ", " << p.y << ") ";
  }
  std::cout << "feasible" << std::endl;
  return path;
}

int main()
{
  std::vector<std::vector<std::vector<bool>>> worlds_tests;
  std::vector<Pose> start_tests;
  std::vector<Pose> goal_tests;
  std::vector<bool> has_solution_tests;
  std::vector<std::vector<bool>> world_state_1 = {{0, 0, 0, 1, 0, 0, 0},
                                                  {0, 0, 0, 1, 0, 0, 0},
                                                  {0, 0, 0, 0, 0, 1, 0},
                                                  {0, 0, 0, 0, 0, 1, 0},
                                                  {0, 0, 0, 1, 1, 1, 0},
                                                  {0, 0, 0, 0, 0, 1, 0},
                                                  {0, 0, 0, 0, 0, 0, 0}};

  std::vector<std::vector<bool>> world_state_2 = {{0, 0, 0, 1, 0, 0, 0},
                                                  {0, 0, 0, 1, 0, 0, 0},
                                                  {0, 0, 1, 0, 0, 1, 0},
                                                  {1, 1, 1, 0, 0, 1, 0},
                                                  {0, 0, 0, 1, 1, 1, 0},
                                                  {0, 0, 0, 0, 0, 1, 0},
                                                  {0, 0, 0, 0, 0, 0, 0}};


  // test 1
  worlds_tests.push_back(world_state_1);
  start_tests.push_back(Pose(2, 0));
  goal_tests.push_back(Pose(6, 6));
  has_solution_tests.push_back(true);

  // test 2
  worlds_tests.push_back(world_state_2);
  start_tests.push_back(Pose(0, 0));
  goal_tests.push_back(Pose(2, 1));
  has_solution_tests.push_back(true);

  // test 3
  worlds_tests.push_back(world_state_2);
  start_tests.push_back(Pose(0, 0));
  goal_tests.push_back(Pose(2, 3));
  has_solution_tests.push_back(false);

  // test 4
  worlds_tests.push_back(world_state_2);
  start_tests.push_back(Pose(3, 0));
  goal_tests.push_back(Pose(0, 0));
  has_solution_tests.push_back(false);

  // test 5
  worlds_tests.push_back(world_state_2);
  start_tests.push_back(Pose(3, 3));
  goal_tests.push_back(Pose(4, 2));
  has_solution_tests.push_back(true);

  // test planners
  std::shared_ptr<discrete_planner::Planner> random_planner = std::make_shared<discrete_planner::RandomPlanner>(discrete_planner::RandomPlanner(100, 0));
  std::shared_ptr<discrete_planner::Planner> optimal_planner = std::make_shared<discrete_planner::OptimalPlanner>(discrete_planner::OptimalPlanner());
  for (size_t i = 0; i < worlds_tests.size(); ++i)
  {
    std::cout << "world_state" << std::endl;
    for (size_t x = 0; x < worlds_tests[i].size(); ++x)
    {
      for (size_t y = 0; y < worlds_tests[i][x].size(); ++y)
      {
        std::cout << worlds_tests[i][x][y] << " ";
      }
      std::cout << std::endl;
    }
    std::cout << "start_state: (" << start_tests[i].x << ", " << start_tests[i].y << ")" << std::endl;
    std::cout << "goal_state: (" << goal_tests[i].x << ", " << goal_tests[i].y << ")" << std::endl;
    std::deque<Pose> random_path = testPlanner(random_planner, worlds_tests[i], start_tests[i], goal_tests[i], has_solution_tests[i]);
    std::deque<Pose> optimal_path = testPlanner(optimal_planner, worlds_tests[i], start_tests[i], goal_tests[i], has_solution_tests[i]);
    assert(optimal_path.size() <= random_path.size());
    std::cout << "Test " << i + 1 << " passed" << std::endl;
    std::cout << std::endl;
  }

  return EXIT_SUCCESS;
}
