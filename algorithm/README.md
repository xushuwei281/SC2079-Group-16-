# SC2079 MDP — Autonomous Path Planner & Simulator

High-performance pathfinding algorithm and interactive 2D arena simulator for SC2079 Multidisciplinary Project (MDP).

- **Kinematics:** Ackermann steering model with forward & reverse maneuvers via **Reeds-Shepp shortest path curves** ($R = 25\text{ cm}$).
- **Optimization:** Exact Traveling Salesperson Problem (TSP) solver using the pairwise Reeds-Shepp distance matrix.
- **Collision Checking:** Continuous Oriented Bounding Box (OBB) separating-axis theorem with a 15 cm safety perimeter.
- **Hardware Mapping:** Discretizes trajectories into verified 5-byte STM32 ASCII instructions (`FC`, `BC`, `FL`, `FR`, `BL`, `BR`).

---

## 1. Quick Start: Interactive Visual Simulator

To launch the 2D Pygame simulator on your laptop/PC:

```bash
python3 algorithm/simulator.py
```
*(Or run `mdp-sim` if aliases are sourced).*

### Simulator Controls
| Control | Action |
|---|---|
| **Left Click + Drag** | Move an obstacle anywhere in the arena |
| **Right Click on Obstacle** | Cycle active target image face ($N \rightarrow E \rightarrow S \rightarrow W$) |
| **SPACE** | Play / Pause robot driving animation |
| **R** | Reset robot back to Start Zone $(20, 20, 90^\circ)$ |
| **C** | Force re-calculation of the TSP Reeds-Shepp plan |
| **ESC / Q** | Quit simulator |

---

## 2. Command-Line Planner Demo

To run a quick trajectory and 5-byte command calculation in the terminal:

```bash
python3 algorithm/planner.py
```

Sample output:
```text
=== Planned Mission with 5 Legs ===
Total Distance: 383.69 cm
Total Movement Commands: 17

Leg 1 -> Obstacle 4 (S face):
  Vantage: x=60.0, y=110.0, th=90°
  Commands: FR027 -> FC075 -> FL027
...
```

---

## 3. Running Unit Tests

```bash
python3 -m unittest discover -s algorithm/test -p "test_*.py"
```

---

## 4. ROS 2 Node Integration

In the live robot environment, the planner runs as a ROS 2 node:
```bash
ros2 run mdp_bringup planner_node
# Or via alias:
mdp-planner
```
- Subscribes to `/android/cmd` for `ALG|<obs_id>,<x>,<y>,<face>|...` obstacle packets.
- Executes legs via `/execute_moves`.
- Publishes detected symbols to `/android/target`.
