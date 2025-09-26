# CSE2001-autonomous-delivery-agent
 Fundamentals of AI and ML Project Based Learning


*Student Name:* Balraj Malviya
*Registration Number:* 24MIM10152

---

## 📌 Project Overview
This project implements an *Autonomous Delivery Agent* that finds the optimal path from a start point to a goal point in a grid-based environment with obstacles.  
The agent uses three different search strategies — *BFS, **UCS, and **A\** — and compares their performance based on path cost, path length, nodes expanded, and execution time.

---

## 🎯 Objectives
- Represent the environment as a 2D grid.
- Randomly generate obstacles while keeping the map solvable.
- Implement and execute:
  - *BFS (Breadth-First Search)* – finds the shortest path by number of steps.
  - *UCS (Uniform Cost Search)* – finds the least-cost path.
  - *A\** – uses heuristic (Manhattan distance) to find the optimal path faster.
- Measure:
  - Path length
  - Total path cost
  - Number of nodes expanded
  - Execution time
- Compare results and identify the most efficient algorithm.

---

## 📂 Repository Structure
.
├── agent.py # Main project file (map generator + algorithms)
├── maps/ # Folder containing generated map files
├── screenshots/ # Screenshots of outputs for BFS, UCS, A*
└── README.md # This file


---

## ⚙ Installation & Setup

*Requirements:*  
- Python 3.8 or later  
- Numpy library  

Install numpy using:
```bash
pip install numpy

🚀 How to Run
1️⃣ Generate Maps

Run once to create the maps:

python agent.py --generate-maps


This will generate three maps:

maps/small_map.txt

maps/medium_map.txt

maps/large_map.txt


2️⃣ Run Algorithms

Run BFS,, or A* on a selected map:

python agent.py --map maps/small_map.txt --planner bfs
python agent.py --map maps/small_map.txt --planner astar

🎯 Conclusion

The project successfully:

Modeled a delivery environment as a grid.

Implemented BFS, and A* search algorithms.

Compared algorithms based on cost, length, and efficiency.

Demonstrated that A* is the best choice for this scenario.

👤 Author

Name:  Balraj Malviya
Reg. No: 24MIM10152
Course: CSA2001 - Fundamentals of AI & ML
Institution: VIT BHOPAL
