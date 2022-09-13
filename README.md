# Dijkstra's algorithm Implementation
## Summary
I implemented an important path-planning algorithm: Dijkstra's algorithm in this project. 

1. Step 1 (step1.cpp)
    * Read in a graph representation from a file.
2. Step 2 (step2.cpp)
    * Use Dijkstra's algorithm to calculate the shortest path from a start to
  goal node.
3. Step 3 (step3.cpp)
    * Use Dijkstra's algorithm to calculate the shortest path when there are
  obstacles.

* Input1: grid map
    * (0,0) (0.5,0) (1,0) (0,0.5) (0.5,0.5) (1,0.5) 
    * 0: 1,0.5 3,0.5 4,0.707107
    * 1: 0,0.5 2,0.5 3,0.707107 4,0.5 5,0.707107
    * 2: 1,0.5 4,0.707107 5,0.5
    * 3: 0,0.5 1,0.707107 4,0.5
    * 4: 0,0.707107 1,0.5 2,0.707107 3,0.5 5,0.5
    * 5: 1,0.707107 2,0.5 4,0.5

* Input2: obstacle file
    * $obstacles
    * 2 14 9

* Input grid map and obstacle file, the best path output:
    * 0 6 13 20 21 22 23 : 3.18062

* See TESTOMG.txt file for other grid maps and obstacle combination
