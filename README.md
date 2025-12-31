# Multi-Objective Path Planning using MOEPSO (ROS) 🚀🧭

This project implements a **Multi-Objective Evolutionary Particle Swarm Optimization (MOEPSO)**–based path planning framework in **ROS**.  
The planner optimizes robot paths by simultaneously considering **path length, smoothness, and safety** in a dynamic environment.

The system is designed to integrate with **ROS navigation, Gazebo simulation, RViz visualization, and dynamic reconfiguration**.

---

## 📁 Project Structure

```bash
.
├── AntColonyAlgorithm_ROS/        # Ant Colony Optimization implementation (ROS)
├── PSO_based_move_base_planner/   # PSO-based move_base planner plugin
├── dynamic_rviz_config/           # Dynamic RViz configuration files
├── dynamic_xml_config/            # Dynamic XML / parameter configs
├── gazebo_plugins/
│   └── pedestrian_sfm_plugin/     # Social Force Model plugin for pedestrians
├── map_plugins/
│   └── voronoi_layer/             # Voronoi costmap layer plugin
├── planner/                       # Core planner modules
├── rviz_plugins/                  # Custom RViz plugins
├── user_config/                   # User-defined parameters
├── moop_1.py                      # MOEPSO implementation (ROS node)
├── sim_env.rar                    # Simulation environment (Gazebo)
├── CMakeLists.txt
└── README.md
```


## 📌 Project Overview
The planner uses MOEPSO to generate an optimized navigation path by balancing:
- Shortest Path – minimizes total travel distance
- Smooth Path – reduces abrupt directional changes
- Safe Path – maximizes distance from obstacles
  
The optimization runs iteratively and publishes the best-found path to ROS topics for visualization and execution.

## 🧠 Algorithm: MOEPSO
### Key Concepts

- Particle Swarm Optimization (PSO)
- Evolutionary mutation and crossover
- Multi-objective fitness aggregation
- Weighted objective optimization

### Fitness Function Components
- Path length cost
- Path smoothness cost
- Obstacle clearance cost

``` bash
Total Fitness = w₁·Safety + w₂·Smoothness + w₃·Path Length
```

## 🧰 Software Requirements
### Operating System
- Ubuntu 18.04 / 20.04

### ROS
- ROS Melodic or ROS Noetic
### Python Dependencies
```bash
pip install numpy
```
### ROS Dependencies
```bash
sudo apt install ros-$ROS_DISTRO-nav-msgs \
                 ros-$ROS_DISTRO-geometry-msgs \
                 ros-$ROS_DISTRO-dynamic-reconfigure \
                 ros-$ROS_DISTRO-rviz \
                 ros-$ROS_DISTRO-gazebo-ros
```

## 🚀 How to Build
Clone the repository into a catkin workspace:
```bash
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws/src
git clone <YOUR_REPOSITORY_URL>
cd ~/catkin_ws
catkin build
source devel/setup.bash
```

## ▶️ How to Run
### 1️⃣ Start ROS Core
```bash
roscore
````

### 2️⃣ Launch Simulation Environment
- After Launching robotino's bringup.launch, Run
- Checkout individual optimization section i.e ACO, PSO, MOEPSO
```bash
roslaunch <simulation_package> <launch_file>.launch
```

### 3️⃣ Run MOEPSO Planner Node
```bash
rosrun <package_name> moop_1.py
```

## 🔧 Dynamic Reconfiguration
The planner supports runtime tuning using dynamic_reconfigure, allowing adjustment of:

- Objective weights
- Particle count
- Iteration limits

Use:
```bash
rosrun rqt_reconfigure rqt_reconfigure
```

## 📜 License
This project is intended for academic and research purposes.
You are free to modify and extend it for learning or experimentation.

## 👤 Author
Shakthi Bala

---

If you want next, I can:
- Add **algorithm flow diagrams**
- Rewrite this as a **research-paper companion repo**
- Clean up `moop.py` (bugs + ROS best practices)
- Create a **demo GIF / RViz screenshot section**

Just say 👍


