# GUI-Based Food Delivery Robot

[![Watch the Video](https://img.youtube.com/vi/RX9A46x0Tgg/0.jpg)](https://www.youtube.com/watch?v=RX9A46x0Tgg)

## 📌 Introduction
This project demonstrates a **GUI-based food delivery robot system**.  
The robot can autonomously navigate in an environment, pick up food orders, and deliver them to specified destinations using ROS 2 and the TurtleBot3 platform.

---

## 🚀 Features
- **GUI Control Panel** for order selection and destination assignment.
- **Autonomous Navigation** using ROS 2 Navigation Stack (Nav2).
- **Simulation Environment** built in Gazebo Harmonic.
- **Obstacle Avoidance** using TurtleBot3’s LiDAR.
- **Multi-Stop Delivery** capability.

---

## 🛠 Requirements
- [ROS 2](https://docs.ros.org/en/humble/index.html) (Humble or newer)  
- [TurtleBot3 Packages](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/)  
- [Gazebo Harmonic](https://gazebosim.org/home)  
- Python 3

---

## 📥 Installation

1. **Create a ROS 2 Workspace**
   ```bash
   mkdir -p goat_ws/src
   cd goat_ws/src
   ```

2. **Clone the Repository**
   ```bash
   git clone https://github.com/vimalgrace/goat_robotics_hotel_robot_project.git
   ```

3. **Build the Workspace**
   ```bash
   cd ..
   colcon build
   ```

4. **Source the Workspace**
   ```bash
   source install/setup.bash
   ```

---

## ▶️ Usage

### 1️⃣ Launch Gazebo Simulation with TurtleBot3
```bash
ros2 launch goat_gazebo simulation.launch.py
```

### 2️⃣ Start the Navigation Stack
```bash
ros2 launch goat_nav2 navigation2.launch.py
```

### 3️⃣ Run the Final Delivery Task
```bash
ros2 run goat_nav2 final_task.py
```


