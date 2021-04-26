import random
'''NEED TO IMPLEMENT:
    "preferred paths": represent paved/unpaved paths
    grids that are not square in shape
    checking to make sure initialized trajectories, start/end goals are valid
    better behavior prediction algorithms (non-random monte carlo?)
    motion planner
    plotting backend
    looking more than one time step ahead
'''
num_to_directions = {
 1: "North",
 2: "Northeast",
 3: "East",
 4: "Southeast",
 5: "South",
 6: "Southwest",
 7: "West",
 8: "Northwest"
 }

directions_to_num = {
        "North": 1, 
        "Northeast": 2,
        "East": 3,
        "Southeast": 4,
        "South": 5,
        "Southwest": 6,
        "West": 7,
        "Northwest": 8
        }

def points_to_direction(point1, point2):
    if point2[0] - point1[0] == 0 and point2[1] - point1[1] == 1:
        return "North"
    if point2[0] - point1[0] == 1 and point2[1] - point1[1] == 1:
        return "Northeast"
    if point2[0] - point1[0] == 1 and point2[1] - point1[1] == 0:
        return "East"
    if point2[0] - point1[0] == 1 and point2[1] - point1[1] == -1:
        return "Southeast"
    if point2[0] - point1[0] == 0 and point2[1] - point1[1] == -1:
        return "South"
    if point2[0] - point1[0] == -1 and point2[1] - point1[1] == -1:
        return "Southwest"
    if point2[0] - point1[0] == -1 and point2[1] - point1[1] == 0:
        return "West"
    if point2[0] - point1[0] == -1 and point2[1] - point1[1] == 1:
        return "Northwest"
    return None

def direction_to_coord(point, direction):
    if direction == "North":
        return (point[0], point[1]+1)
    if direction == "Northeast":
        return (point[0]+1, point[1]+1)
    if direction == "East":
        return (point[0]+1, point[1])
    if direction == "Southeast":
        return (point[0]+1, point[1]-1)
    if direction == "South":
        return (point[0], point[1]-1)
    if direction == "Southwest":
        return (point[0]-1, point[1]-1)
    if direction == "West":
        return (point[0]-1, point[1])
    if direction == "Northwest":
        return (point[0]-1, point[1] + 1)
    return None

def check_bounds(point, grid_size):
    if point[0] < 0 or point[0] > grid_size or point[1] < 0 or point[1] > grid_size:
        return False
    return True

# grid_size is an integer determining the side length of the grid where the 
#   bottom left corner is (0,0)
# num_agents is an integer determining the number of agents
# agent_positions is a list of pairs determining the initial position of the agents
# agent_trajectories dictionary mapping agent IDs to a trajectory represented as a list
class Scenario(grid_size, agent_trajectories, robot_start, robot_goal):
    def __init__(self, grid_size, num_agents, agent_trajectories, robot_start, robot_goal):
        self.grid_size = grid_size
        self.num_agents = len(agent_trajectories)
        self.agent_trajectories = agent_trajectories
        self.robot_start = robot_start
        self.robot_goal = robot_goal
        
    def get_agent_trajectories(self):
        return self.agent_trajectories
    
    def get_robot_start(self):
        return self.robot_start
    
    def get_robot_goal(self):
        return self.robot_goal
    
    def get_grid_size(self):
        return self.grid_size
        
# creates an environment with agent positions plotted
# agent_trajectories dictionary mapping agent IDs to a trajectory represented as a 
# list
class Environment(agent_trajectories, robot_pos, robot_goal, grid_size):
    def __init__(self, agent_trajectories, robot_start, robot_goal, grid_size):
        self.agent_trajectories = agent_trajectories
        self.robot_pos = robot_pos
        self.robot_goal = robot_goal
        self.grid_size = grid_size
        self.time = 0

    def update_time(self):
        self.time += 1
        
    def get_time(self):
        return self.time
    
    def update_agents(self):
        agent_positions = []
        for agent, position in self.agent_trajectories:
            agent_positions.append((agent,position.get(time)))
        return agent_positions
    
    def get_robot_pos(self):
        return self.robot_pos
    
    def get_robot_goal(self):
        return self.robot_goal
    
    def get_grid_size(self):
        return self.grid_size
    
class BehaviorPrediction(agent_pos, robot_pos):
    def __init__(self, agent_pos, robot_pos):
        self.robot_pos = robot_pos
        self.agent_trajectories = {}
        # track agent history
        for agents in agent_pos:
            self.agent_trajectories[agents[0]] = [agents[1]]
        
    def update(self, environment):
        environment.update_time()
        new_agents = environment.update_agents()
        for agent in new_agents:
            self.agent_trajectories[agent[0]].append(agent[1])
     
    # only looking one step ahead right now
    def predict(self, environment):
        predicted = []
        if environment.get_time < 2:
            for agent in self.agent_trajectories:
                random_choice = []
                current_direction = points_to_direction(self.agent_trajectories[agent][-1], self.agent_trajectories[agent][-2])
                left = num_to_directions[(directions_to_num[current_direction]-1)%8]
                right = num_to_directions[(directions_to_num[current_direction]+1)%8]
                if check_bounds(direction_to_coord(self.agent_trajectories[agent][-1], current_direction), environment.get_grid_size()):
                    random_choice += [current_direction] * 5
                if check_bounds(direction_to_coord(self.agent_trajectories[agent][-1], left), environment.get_grid_size()):
                    random_choice += [left] * 2
                if check_bounds(direction_to_coord(self.agent_trajectories[agent][-1], right), environment.get_grid_size()):
                    random_choice += [right] * 2
                random_direction = random.choice(random)
                random_coord = direction_to_coord(self.agent_trajectories[agent][-1], random_direction)
                predicted.append((agent, random_coord))
        return predicted
                    


    
    
    

