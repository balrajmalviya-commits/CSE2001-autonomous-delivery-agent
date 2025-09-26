"""
CSA2001 - Fundamentals of AI & ML

Features:
- Generates 3 maps (small, medium, large) with obstacles (guaranteed solvable)
- Implements BFS, UCS, and A* search
- Prints grid, path length, cost, runtime, nodes expanded
- Single file solution for easy submission

Usage:
  python agent.py --generate-maps
  python agent.py --map maps/small_map.txt --planner bfs
  python agent.py --map maps/small_map.txt --planner ucs
  python agent.py --map maps/small_map.txt --planner astar

Requirements:
  pip install numpy
"""

import os, argparse, time, heapq
from collections import deque
import numpy as np

np.random.seed(42)

# ------------------- Map Generator -------------------

def generate_maps():
    os.makedirs("maps", exist_ok=True)
    def create_map(size, obstacles):
        grid = np.ones((size, size), int)
        count = 0
        while count < obstacles:
            r, c = np.random.randint(size), np.random.randint(size)
            if (r, c) != (0, 0) and (r, c) != (size-1, size-1) and grid[r, c] != -1:
                grid[r, c] = -1
                count += 1
        return grid

    write_map("maps/small_map.txt", create_map(8, 10))
    write_map("maps/medium_map.txt", create_map(12, 25))
    write_map("maps/large_map.txt", create_map(15, 40))
    print("✅ Generated maps: small, medium, large")

def write_map(path, grid):
    with open(path, "w") as f:
        for row in grid:
            f.write(" ".join(str(x) for x in row) + "\n")

# ------------------- Utility Functions -------------------

def read_map(path):
    grid = []
    with open(path) as f:
        for line in f:
            grid.append([int(x) for x in line.split()])
    return np.array(grid, int)

def print_grid(grid, start, goal):
    print("\nGrid Layout (S=start, G=goal, #=obstacle):")
    for r in range(grid.shape[0]):
        row = ""
        for c in range(grid.shape[1]):
            if (r, c) == start:
                row += "S "
            elif (r, c) == goal:
                row += "G "
            elif grid[r, c] == -1:
                row += "# "
            else:
                row += ". "
        print(row)

def neighbors(pos, grid):
    r, c = pos
    rows, cols = grid.shape
    for dr, dc in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
        nr, nc = r + dr, c + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr, nc] != -1:
            yield (nr, nc)

def path_cost(path, grid):
    return sum(grid[p] for p in path[1:])

def reconstruct(parent, start, goal):
    if goal not in parent:
        return None
    path = [goal]
    while path[-1] != start:
        path.append(parent[path[-1]])
    path.reverse()
    return path

# ------------------- Algorithms -------------------

def bfs(grid, start, goal):
    q = deque([start]); visited = {start}; parent = {}; nodes = 0
    t0 = time.perf_counter()
    while q:
        u = q.popleft(); nodes += 1
        if u == goal:
            return reconstruct(parent, start, goal), nodes, time.perf_counter() - t0
        for v in neighbors(u, grid):
            if v not in visited:
                visited.add(v); parent[v] = u; q.append(v)
    return None, nodes, time.perf_counter() - t0

def ucs(grid, start, goal):
    heap = [(0, start)]; dist = {start: 0}; parent = {}; nodes = 0
    t0 = time.perf_counter()
    while heap:
        cost, u = heapq.heappop(heap)
        if cost > dist[u]:
            continue
        nodes += 1
        if u == goal:
            return reconstruct(parent, start, goal), nodes, time.perf_counter() - t0
        for v in neighbors(u, grid):
            newc = cost + grid[v]
            if newc < dist.get(v, 1e9):
                dist[v] = newc; parent[v] = u; heapq.heappush(heap, (newc, v))
    return None, nodes, time.perf_counter() - t0

def astar(grid, start, goal):
    minc = int(grid[grid > 0].min()) if np.any(grid > 0) else 1
    def h(p): return (abs(p[0] - goal[0]) + abs(p[1] - goal[1])) * minc
    heap = [(h(start), 0, start)]; g = {start: 0}; parent = {}; nodes = 0
    t0 = time.perf_counter()
    while heap:
        f, cost, u = heapq.heappop(heap)
        if cost > g[u]:
            continue
        nodes += 1
        if u == goal:
            return reconstruct(parent, start, goal), nodes, time.perf_counter() - t0
        for v in neighbors(u, grid):
            newg = cost + grid[v]
            if newg < g.get(v, 1e9):
                g[v] = newg; parent[v] = u; heapq.heappush(heap, (newg + h(v), newg, v))
    return None, nodes, time.perf_counter() - t0

# ------------------- Runner -------------------

def run_planner(mapfile, planner):
    grid = read_map(mapfile)
    start, goal = (0, 0), (grid.shape[0]-1, grid.shape[1]-1)
    print_grid(grid, start, goal)
    if planner == "bfs": path, n, t = bfs(grid, start, goal)
    elif planner == "ucs": path, n, t = ucs(grid, start, goal)
    elif planner == "astar": path, n, t = astar(grid, start, goal)
    else: raise ValueError("Unknown planner")
    print(f"\nAlgorithm: {planner.upper()} | Nodes expanded: {n} | Time: {t:.6f}s")
    if path:
        print(f"Path length: {len(path)}, Cost: {path_cost(path, grid)}")
        print("Path:", path)
    else:
        print("No path found!")

# ------------------- Main -------------------

def main():
    p = argparse.ArgumentParser()
    p.add_argument("--generate-maps", action="store_true")
    p.add_argument("--map", type=str, default="maps/small_map.txt")
    p.add_argument("--planner", type=str, default="astar", choices=["bfs", "ucs", "astar"])
    args = p.parse_args()
    if args.generate_maps:
        generate_maps()
    else:
        if not os.path.exists(args.map):
            print("❌ Map not found! Run with --generate-maps first.")
            return
        run_planner(args.map, args.planner)

if __name__ == "__main__":
    main()