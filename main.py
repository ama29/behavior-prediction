# -*- coding: utf-8 -*-
"""
Created on Wed Jun 16 02:52:44 2021

@author: maail
"""
import random
from extended_kalman import plot_extended_kalman
from particle_filter import plot_particle

# Test cases assume robot is starting at (0,0) and moving in positive horizontal direction (east)
def main():
    print(__file__ + " start!!")

    prediction_input = input("What kind of trajectory prediction is preferred? (extended kalman, cubature kalman, ensemble kalman, particle, histogram)")
    test_num = input("Which test case do you want to run?")
   
    if prediction_input == "extended kalman":
        prediction = plot_extended_kalman
    elif prediction_input == "particle":
        prediction = plot_particle
    else:
        return "prediction input not valid"
    
    # Test 1: single stationary agent
    # Test prediction around stationary obstacles

    if int(test_num) == 1:
        test1 = []
        velocity1 = 0
        for i in range(100):
            test1.append([[10],[0],[0],[velocity1]])
        agents = [test1]
        prediction(agents)
        return
    
    # Test 2: multiple stationary agents clustered together
    # Test prediction around stationary obstacles
    if int(test_num) == 2:
        test1 = []
        for i in range(100):
            test1.append([[10],[0],[0],[0]])
        test2 = []
        for i in range(100):
            test2.append([[10],[1],[0],[0]])
        test3 = []
        for i in range(100):
            test3.append([[9],[0],[0],[0]])
        agents = [test1, test2, test3]     
        prediction(agents)
        return
    
    # Test 3: multiple stationary agents spread apart
    # Test prediction around stationary obstacles
    if int(test_num) == 3:
        test1 = []
        for i in range(100):
            test1.append([[10],[10],[0],[0]])
        test2 = []
        for i in range(100):
            test2.append([[10],[0],[0],[0]])
        test3 = []
        for i in range(100):
            test3.append([[0],[10],[0],[0]]) 
        agents = [test1, test2, test3]
        prediction(agents)
        return
    
    # Test 4: single linear agent, constant velocity, same direction as robot
    # Test merging/following
    if int(test_num) == 4:
        test1 = []
        velocity1 = 1
        for i in range(100):
            test1.append([[i],[1],[0],[velocity1]])
        agents = [test1]
        prediction(agents)
        return
    
    # Test 5: single linear agent, constant velocity, opposite direction as robot
    # Test collision avoidance
    if int(test_num) == 5:
        test1 = []
        velocity1 = 1
        for i in range(100):
            test1.append([[50-i],[0],[0],[velocity1]])
        agents = [test1]
        prediction(agents)
        return
    
    # Test 6: multiple linear agents, constant velocity, same direction as robot
    # Test merging/following
    if int(test_num) == 6:
        test1 = []
        for i in range(100):
            test1.append([[i],[1],[0],[1]])
        test2 = []
        for i in range(100):
            test2.append([[i],[1],[0],[1]])
        test3 = []
        for i in range(100):
            test3.append([[i],[1],[0],[1]])
        agents = [test1, test2, test3]     
        prediction(agents)
        return
    
    # Test 7: multiple linear agents, constant velocity, opposite direction as robot
    # Test collision avoidance
    if int(test_num) == 7:
        test1 = []
        for i in range(100):
            test1.append([[60-i],[0],[0],[1]])
        test2 = []
        for i in range(100):
            test2.append([[60-i],[0],[0],[1]])
        test3 = []
        for i in range(100):
            test3.append([[60-i],[0],[0],[1]])
        agents = [test1, test2, test3]     
        prediction(agents)
        return
    
    # Test 8: single linear agent, constant velocity, orthogonal intersection with robot
    # Test collision avoidance
    if int(test_num) == 8:
        test1 = []
        velocity1 = 1
        for i in range(100):
            test1.append([[50],[50-i],[0],[velocity1]])
        agents = [test1]
        prediction(agents)
        return
    
    # Test 9: multiple linear agents, constant velocity, orthogonal intersection with robot
    # Test collision avoidance
    if int(test_num) == 9:
        test1 = []
        for i in range(100):
            test1.append([[50],[50-i],[0],[1]])
        test2 = []
        for i in range(100):
            test2.append([[50],[50-i],[0],[1]])
        test3 = []
        for i in range(100):
            test3.append([[50],[50-i],[0],[1]])
        agents = [test1, test2, test3]     
        prediction(agents)
        return
    
     # Test 10: single linear agent, nonconstant velocity, same direction as robot
     # Test following/merging
    if int(test_num) == 10:
        test1 = []
        velocity1 = 1
        for i in range(100):
            rand = random.randint(-1,1)
            velocity1 = max(0, rand+velocity1)
            test1.append([[i+5],[1],[0],[velocity1]])
        agents = [test1]
        prediction(agents)
        return
    
    # Test 11: multiple linear agents, nonconstant velocity, same direction as robot
    # Test following/merging
    if int(test_num) == 11:
        test1 = []
        velocity1 = 1
        for i in range(100):
            rand = random.randint(-1,1)
            velocity1 = max(0, rand+velocity1)
            test1.append([[i+5],[1],[0],[velocity1]])
        test2 = []
        velocity2 = 1
        for i in range(100):
            rand = random.randint(-1,1)
            velocity2 = max(0, rand+velocity2)
            test1.append([[i+5],[1],[0],[velocity2]])
        agents = [test1, test2]
        prediction(agents)
        return
    
    return "Error in generating prediction trajectory"


if __name__ == '__main__':
    main()
    
