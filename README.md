# behavior-prediction

This repo is based off the AtsushiSakai code that compiles and visualizes several robotics algorithms. The purpose of this repo is to provide a testing benchmark for The main file (main.py) provides users the option of visualizing several scenarios involving agents. 

Test 1: Singular stationary agent (testing prediction involving a stationary obstacle)
Test 2: Multiple stationary agents clustered together (testing prediction involving multiple stationary obstacles)
Test 3: Multiple stationary agents spread apart (testing prediction involving multiple stationary obstacles)
Test 4: Single agent, constant velocity, linear path, same direction as robot (testing merging/following)
Test 5: Single agent, constant velocity, linear path, opposite direction as robot (testing collision avoidance)
Test 6: Multiple agents, constant velocity, linear path, same direction as robot (testing merging/following)
Test 7: Multiple agents, constant velocity, linear path, opposite direction as robot (testing collision avoidance)
Test 8: Single agent, constant velocity, linear path, orthogonal intersection (testing collision avoidance)
Test 9: Multiple agents, constant velocity, linear path, orthogonal intersection (testing collision avoidance)
Test 10: Single agent, nonconstant velocity, linear path, same direction as robot (testing following/merging)
Test 11: Multiple agents, nonconstant velocity, linear path, same direction as robot (testing following/merging)

Users can choose one of the 11 tests to visualize, along with a selection between either a Kalman extended filter or particle filter to predict trajectories. One of the primary modifications to this code from the original AtsushiSakai repo is the ability to visualize several agents on a single plot. Additionally, the various benchmarks allow for a user to understand scenarios that their robot may find themselves in. Comments about the visualization themselves (what the colors represent) can be found int the documentation of the code itself.

Future modifications of the code can focus on a few major aspects. First, integration with a motion planner may necessitate use of a CSV to account for data. Second, tests can be more comprehensive in accounting for a greater breadth of scenarios, including hard coding bounds (to represent walls or off-road terrain), including non-linear paths, agents with different velocities and starting positions, and accounting for "socially acceptable" trajectories (bubbles around agents). Third, a greater variety of trajectory prediction algorithms can be applied, including cubature kalman, ensemble kalman, and histogram filters.
