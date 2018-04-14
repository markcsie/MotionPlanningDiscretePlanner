**Run**:
```bash
mkdir build
cd build
cmake .. 
make
./test/test_planners
```

**Implementation**:  
Random Planner: As described by the specification.  
Optimal Planner: Standard BFS search.  

**Time Complexity**:  
Random Planner: O(k), where k is the max_step_number.  
Optimal Planner: O(mn), where m and n are the number rows and columns respectively.  
