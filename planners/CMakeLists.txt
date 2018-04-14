set(PLANNERS_SRC
  src/random_planner
  src/optimal_planner

  include/planner
  include/random_planner
  include/optimal_planner
  include/pose
  include/util
)

add_library(MotionPlanningDiscretePlanner ${PLANNERS_SRC})
target_include_directories (MotionPlanningDiscretePlanner PUBLIC include)
