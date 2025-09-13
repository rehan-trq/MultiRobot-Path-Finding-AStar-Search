import heapq
import random
import re

class Robot:
    def __init__(self, robot_id, start_coord, target_coord):
        self.id = robot_id
        self.start = start_coord
        self.target = target_coord
        self.route = []           # Remaining moves to execute.
        self.complete_route = []  # Stores the full optimal path found.
        self.position = start_coord
        self.reached = False
        self.failures = 0         # Count of consecutive failed moves.

class DynamicAgent:
    def __init__(self, positions, time_order):
        # Pair each time with its position and sort based on time.
        sorted_pairs = sorted(zip(time_order, positions))
        self.positions = [pos for _, pos in sorted_pairs]
        self.n = len(self.positions)
        # Cycle length: moving forward then reversing (excluding duplicate endpoints)
        self.cycle = 2 * (self.n - 1) if self.n > 1 else 1

    def get_position(self, time):
        if self.n == 0:
            return None
        t_mod = time % self.cycle
        if t_mod < self.n:
            return self.positions[t_mod]
        else:
            return self.positions[self.cycle - t_mod]

def read_grid(file_path):
    with open(file_path, "r") as f:
        content = f.readlines()
    rows = int(content[0].strip())  # Number of rows in the grid.
    grid_layout = [list(line.rstrip("\n")) for line in content[1:rows+1]]
    return grid_layout

def load_robots(file_path):
    robots_list = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
    for idx, line in enumerate(lines):
        # Expected format: "Robot 1: Start (11, 22) End (19, 14)"
        m = re.search(r"Start \((\d+), (\d+)\) End \((\d+), (\d+)\)", line)
        if m:
            start_pos = (int(m.group(1)), int(m.group(2)))
            end_pos = (int(m.group(3)), int(m.group(4)))
            robots_list.append(Robot(idx, start_pos, end_pos))
    return robots_list

def load_dynamic_agents(file_path):
    agents = []
    with open(file_path, 'r') as f:
        lines = f.readlines()
    for line in lines:
        # Use regex to capture all positions and time stamps non-greedily.
        m = re.search(r"Agent \d+: \[\((.*?)\)\] at times \[(.*?)\]", line)
        if not m:
            print(f"ERROR: Could not parse agent line: {line.strip()}")
            continue
        pos_str = m.group(1)
        time_str = m.group(2)
        # Split positions and convert to tuple of ints.
        pos_tokens = pos_str.split("), (")
        positions = [tuple(map(int, token.replace("(", "").replace(")", "").split(", "))) for token in pos_tokens]
        times = list(map(int, time_str.split(", ")))
        if len(positions) != len(times):
            print(f"ERROR: Mismatch in number of positions and times for agent: {line.strip()}")
            continue
        agent = DynamicAgent(positions, times)
        agents.append(agent)
    return agents

def get_dynamic_positions(dynamic_agents, time):
    positions = set()
    for agent in dynamic_agents:
        pos = agent.get_position(time)
        if pos is not None:
            positions.add(pos)
    return positions

def manhattan(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])

def a_star_search(start, goal, grid, dynamic_agents, start_time=0):
    # Each state: (estimated_total_cost, current_position, current_time, path_taken)
    open_set = [(manhattan(start, goal), start, start_time, [start])]
    heapq.heapify(open_set)
    explored = set()  # Contains (position, time) pairs.
    moves = [(0, 1), (1, 0), (0, -1), (-1, 0)]  # Allowed moves (no waiting).
    max_iters = 10000
    iters = 0

    while open_set:
        iters += 1
        if start != (169, 12) and iters > max_iters:
            return []
        est_total, current, time, path = heapq.heappop(open_set)
        if current == goal:
            print(f"Path Found: {path} at time {time}")
            return path
        state = (current, time)
        if state in explored:
            continue
        explored.add(state)
        for dx, dy in moves:
            next_cell = (current[0] + dx, current[1] + dy)
            # Check grid boundaries.
            if not (0 <= next_cell[0] < len(grid) and 0 <= next_cell[1] < len(grid[0])):
                continue
            # Skip static obstacles (assuming obstacles are marked as 'X').
            if grid[next_cell[0]][next_cell[1]] == 'X':
                continue
            # Avoid dynamic obstacles: next_cell must not be occupied at time+1.
            if next_cell in get_dynamic_positions(dynamic_agents, time + 1):
                # Debug:
                print(f"DEBUG: Dynamic obstacle at {next_cell} at time {time + 1}")
                print(get_dynamic_positions(dynamic_agents, time + 1))
                continue
            new_time = time + 1
            new_path = path + [next_cell]
            cost_so_far = len(new_path)
            total_cost = cost_so_far + manhattan(next_cell, goal)
            heapq.heappush(open_set, (total_cost, next_cell, new_time, new_path))
    print("No Path Found (Goal unreachable within explored state space).")
    return []

def simulate_robots(robots, dynamic_agents, grid):
    current_time = 0
    active = set(robots)
    max_steps = 10000

    while active and current_time < max_steps:
        occupied = set()
        for robot in list(active):
            if not robot.route:
                # Compute a path using A* from current position to target.
                route = a_star_search(robot.position, robot.target, grid, dynamic_agents, current_time)
                if route:
                    robot.route = route[:]         # Copy for execution.
                    robot.complete_route = route[:]  # Save the full optimal path.
                   # print(f"DEBUG: Robot {robot.id} route assigned: {robot.complete_route}")
            if not robot.route:
                robot.failures += 1
                if robot.failures > 20:
                    print(f"Robot {robot.id} is stuck and removed.")
                    active.remove(robot)
                continue
            # Execute the next move.
            next_move = robot.route.pop(0)
            if next_move in occupied:
                # Recompute path if collision is detected.
                robot.route = a_star_search(robot.position, robot.target, grid, dynamic_agents, current_time)
                continue
            occupied.add(next_move)
            robot.position = next_move
            if robot.position == robot.target:
                robot.reached = True
                active.remove(robot)
        current_time += 1
    print(f"\nSimulation ended at time step {current_time}")

def main():
    grid = read_grid('D:\Semester 6\AI\Assignment1\data0.txt')
    print(f"Grid size: {len(grid)} rows, {max(len(row) for row in grid)} columns")

    robots = load_robots('D:\Semester 6\AI\Assignment1\Robots0.txt')
    dynamic_agents = load_dynamic_agents('D:\Semester 6\AI\Assignment1\Agent0.txt')

    # Debug: Print grid layout.
   # print("\n--- Grid ---")
   # for row in grid:
   #     print("".join(row))

    # Debug: Print robots information.
    print("\n--- Robots ---")
    for r in robots:
        print(f'Robot {r.id}: Start {r.start}, Goal {r.target}')

    # Debug: Print dynamic agents' cycle information.
    print("\n--- Dynamic Agents ---")
    for idx, agent in enumerate(dynamic_agents):
        print(f"Agent {idx + 1}: Cycle length {agent.cycle}, Path: {agent.positions}")

    simulate_robots(robots, dynamic_agents, grid)

    for r in robots:
        total_time = len(r.complete_route) - 1 if r.complete_route else -1
        print(f"\nRobot {r.id + 1} Full Path: {r.complete_route}")
        print(f"Robot {r.id + 1} Total Time: {total_time}")

if __name__ == "__main__":
    main()