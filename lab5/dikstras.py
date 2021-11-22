from heapq import *
import ik 
import fk
import specs
import stuff
import numpy as np

to_check = [(1,0), (-1,0), (0,1), (0,-1)]

open = 0
occupied = 1
goal = 3

def get_value(map, curr_pos, prev):
    check_x, check_y = curr_pos[0], curr_pos[1]
    if check_x < 0 or check_y < 0 or check_x >= len(map[0]) or check_y >= len(map):
        return occupied 
    elif abs(check_x) + abs(check_y) == 2 and (map[check_y][prev[0]] == occupied or map[prev[1]][check_x] == occupied):
        return occupied 
    else: 
        return map[check_y][check_x]

def get_path(map, start, start_thetas, pos_to_t):
    queue = []
    height = len(map)
    width = len(map[0])
    previous = {}
    goal_pos = None
    heappush(queue, (0, 0, start, None, start_thetas))
    on = 0
    while(len(queue) != 0):
        on += 1
        popped = heappop(queue)
        _on, dist, curr_pos, prev, curr_thetas = popped
        if curr_pos in previous:
            continue
        value = get_value(map, curr_pos, prev)
        if value != occupied:
            previous[curr_pos] = (prev, curr_thetas)
            if value == goal:
                goal_pos = curr_pos
                break
            else: 
                for diff in to_check:
                    new_pos = (curr_pos[0] + diff[0], curr_pos[1] + diff[1])
                    new_thetas = ik.get_thetas_persistent(pos_to_t(new_pos), curr_thetas, persistent_tries=4)
                    if not(new_thetas is None):
                        cost = specs.angle_cost(new_thetas - curr_thetas)
                        # cost = 1
                        heappush(queue, (0, dist + cost, new_pos, curr_pos, new_thetas))
    assert(goal_pos != None)
    rconfigurations = []
    # for testing
    map_positions = []
    pos_on = goal_pos
    while(pos_on != start):
        prev, configuration = previous[pos_on]
        rconfigurations.append(configuration)
        map_positions.append(pos_on)
        pos_on = prev 

    configurations = list(reversed(rconfigurations))
    total_cost = 0
    for i in range(len(configurations) - 1):
        total_cost += specs.angle_cost(configurations[i + 1] - configurations[i])

    return configurations, total_cost, previous

map = [[0] * 3 for _ in range(3)]
map[2][2] = goal
start = (0, 0)

x_min = -200 
x_max = 0 
y_min = -50 
y_max = 50
width = len(map[0])
height = len(map)
def pos_to_t(pos):
    return stuff.shift_to_t([pos[0] * (x_max - x_min) / width + x_min, pos[1] * (y_max - y_min) / height + y_min, 0])
start_thetas = ik.get_thetas_persistent(pos_to_t(start), np.array([0, -0.8, 0, 0, 0, 0]))
print(get_path(map, start, start_thetas, pos_to_t))