# 🚀 Light-Seeking Robot Simulation in ROS  

## 📌 Project Overview  
The **Light-Seeking Robot** is a simulation-based project developed using **ROS (Robot Operating System)** and **Gazebo**. The robot autonomously navigates towards a light source using sensors while avoiding obstacles. This project explores fundamental robotics concepts such as **sensor integration, motor control, path planning, and decision-making logic** within a simulated environment.  

---

## 🎯 Motivation  
Autonomous robots require efficient light-seeking abilities for various applications:  
- 🔥 **Rescue Missions** – Navigating towards emergency beacons in disaster scenarios.  
- ☀ **Energy Efficiency** – Seeking solar charging stations for renewable energy use.  
- 🌍 **Exploration & Navigation** – Identifying well-lit areas in unknown environments.  

Many existing solutions are **complex and expensive**, making it necessary to develop a **cost-effective and adaptable alternative**.  

---

## 🎯 Objectives  
✅ Simulate a **light-seeking robot** in **ROS & Gazebo**.  
✅ Implement **light sensor-based navigation** using a **differential intensity method**.  
✅ Develop **basic path planning & obstacle avoidance**.  
✅ Visualize robot behavior using **Gazebo**.  

---

## 👥 Project Team  
👤 **Tinsae Tadesse** – UGR/3556/14  
👤 **Sifan Fita** – UGR/8856/14  
👤 **Sura Itana** – UGR/2347/14  

---

### **📥 Installation Steps**  
```bash
# Clone the repository
git clone https://github.com/your-username/light-seeking-robot.git

# Navigate to the workspace
cd light-seeking-robot

source /opt/ros/jazzy/setup.bash
# Build the package
colcon build
# Source the setup file
source install/setup.bash
ros2 launch light_seeking_robot light_seeking.launch.py

```
