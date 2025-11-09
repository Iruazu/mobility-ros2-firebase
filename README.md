# 🤖 ROS2–Firebase Mobility Platform  
**Connecting Robotics and Cloud**

[![Work in Progress](https://img.shields.io/badge/status-work%20in%20progress-yellow)](https://github.com/Iruazu/mobility-ros2-firebase)
[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue)](https://docs.ros.org/en/humble/)
[![Firebase](https://img.shields.io/badge/Firebase-Firestore%20%26%20Realtime%20DB-orange)](https://firebase.google.com/)
[![License](https://img.shields.io/badge/license-Apache%202.0-green)](LICENSE)

---

## 📖 Overview

This repository presents an experimental platform for **real-time autonomous mobility dispatch and control**, integrating **ROS2 Humble**, **Firebase (Firestore & Realtime Database)**, and **Gazebo simulation**.  
The system enables cloud-based goal distribution to simulated robots — bridging the gap between physical robotics and cloud infrastructure.

### Purpose
- Demonstrate bidirectional communication between **cloud services (Firebase)** and **robotics middleware (ROS2)**  
- Validate GPS-to-simulation coordinate conversion for real-world deployment  
- Build a foundation for **fleet-management systems** in personal-mobility applications  

---

## 💡 Motivation / Development Background

I am **Yugo Obana**, a second-year undergraduate student in Mechanical Systems Engineering at Utsunomiya University.  
This project began as a **commissioned initiative within the Intelligent Robotics Lab**, where I was given the opportunity to prototype a **real-world mobility system** as part of a research-driven exploration.  
Building on that foundation, I independently evolved it into a **cloud-integrated ROS2 platform**, exploring how robotics, cloud infrastructure, and intelligent automation can be seamlessly connected.  
Through this initiative, I aim to bridge mechanical systems and scalable cloud architectures — pursuing the role of a **Mobility × Cloud × AI Architect**.

### Why This Project
- **Early Research Exploration** – Gained hands-on experience in integrating physical and digital systems before formal lab assignment  
- **Innovation from Constraints** – Designed a full **simulation-to-cloud pipeline** using Gazebo + Firebase in the absence of real hardware  
- **Iterative System Design** – Migrated from Docker + Nav2 to **Ubuntu-native execution** to improve real-time performance under GPU limits  
- **AI-Assisted Development** – Used AI tools (Claude, GPT) to refine structure, documentation, and architecture — aligning with modern engineering workflows  

---

### Career Vision

This project embodies my aspiration to become a **Mobility × Cloud × AI Architect** —  
an engineer who designs scalable, intelligent mobility systems by connecting **robotics**, **cloud platforms**, and **real-world data**.  
Rather than focusing narrowly on one discipline, I aim to **bridge mechanical engineering and software architecture**, leading cross-domain solutions that bring automation closer to everyday life.  
Ultimately, I hope to contribute to **global R&D and system innovation** within organizations such as **Bosch**, **AWS**, or emerging robotics ventures.

> _“This project is my bridge between learning and contribution.”_

---

## 🛠️ Technology Stack

**Core Components**
- ROS2 Humble – robotics middleware for node coordination  
- Gazebo 11 – 3D robotics simulator with physics engine  
- Firebase Admin SDK – cloud backend for real-time data synchronization  
  - Firestore for position/status persistence  
  - Realtime Database for goal distribution and polling  
- Python 3.10+ – primary development language  
- TurtleBot3 Waffle – simulated differential-drive platform  

**System Requirements**
- Ubuntu 22.04 LTS (native execution)  
- 8 GB RAM minimum  
- GPU recommended for Gazebo  
- Internet connection for Firebase access  

**Design Philosophy**
- Lightweight architecture (no Nav2 dependency; direct `cmd_vel` control)  
- Dual Firebase integration – Realtime DB (2 Hz) + Firestore (5 Hz)  
- Custom world model – 3D replica of Utsunomiya University Yoto Campus created with Blender + OpenStreetMap  

---

## 🏗️ Architecture Overview

```
┌─────────────────┐
│   Web UI / App  │  (Future: React + Google Maps)
└────────┬────────┘
         │ HTTP/REST
         ▼
┌─────────────────────────────────────┐
│  Firebase Realtime Database         │
│  Path: robot/goal                   │
│  Data: {x: lat, y: lng}             │
└────────┬────────────────────────────┘
         │ Polling (2Hz)
         ▼
┌─────────────────────────────────────┐
│  ROS2 Node: simple_goal_navigator   │
│  ┌───────────────────────────────┐  │
│  │ • GPS → Gazebo conversion     │  │
│  │ • PID-style cmd_vel control   │  │
│  │ • Odometry monitoring         │  │
│  └───────────────────────────────┘  │
└────────┬─────────────┬──────────────┘
         │             │
         ▼             ▼
┌─────────────┐ ┌─────────────────────┐
│   Gazebo    │ │  Firebase Firestore │
│  Simulator  │ │  Collection: robots │
│             │ │  Doc: robot_001     │
│  • TurtleBot│ │  Data:              │
│  • Campus   │ │   - position (GPS)  │
│    Model    │ │   - status          │
│  • Physics  │ │   - timestamp       │
└─────────────┘ └─────────────────────┘
         ▲
         │ /odom (5Hz)
         │ /cmd_vel (10Hz)
         │
    ROS2 Topics
```


---

### Data Flow
1. **Goal Distribution** – Web interface writes GPS coordinates to the Realtime Database  
2. **Coordinate Conversion** – `simple_goal_navigator` converts GPS to Gazebo map frame  
3. **Motion Control** – Direct `cmd_vel` publishing with proportional angular/linear control  
4. **Position Synchronization** – Odometry data converted back to GPS and synced to Firestore (5 Hz)  
5. **Status Update** – Goal is deleted upon arrival (tolerance: ±0.3 m)  

---

## ✨ Key Features

### 1. Nav2-Free Control Design
- Eliminates heavy navigation stacks for faster response times  
- Optimized for controlled environments such as campuses and warehouses  
- Direct `cmd_vel` control with proportional angular and linear tuning  

### 2. Dual Firebase Integration
- **Realtime Database**: Low-latency goal polling (2 Hz) with hash-based duplicate prevention  
- **Firestore**: Reliable position persistence using GeoPoint data  

### 3. GPS–Gazebo Coordinate Conversion

- Reference point: Gazebo (0,0) = GPS (36.551291, 139.928716)
- Conversion factors for Japan latitude (~36°N)
  meters_per_lat = 111320.0
  meters_per_lng = 91290.0


### 4. Custom Campus World Model

- 3D model of Utsunomiya University Yoto Campus
- Built with Blender + BlenderGIS + OpenStreetMap data
- Exported in COLLADA (.dae) with texture mapping
    

### 5. Real-Time Position Synchronization

- Odometry → GPS conversion at 5 Hz  
- Firestore GeoPoint updates with server timestamps  
- Error handling tuned for Firebase free-tier quota (20,000 writes/day)
    

---

## 📊 Project Evolution

|Version|Key Features|Environment|
|---|---|---|
|**v0.1**|Initial Docker + Nav2 integration; basic Firebase sync|Docker + WSLg|
|**v0.2**|Multi-robot support; infinite-loop prevention (hash-based)|Docker + Nav2|
|**v0.3**|Ubuntu-native migration; Nav2-free design; custom campus model|Ubuntu 22.04 (native)|

_See [`README_legacy.md`](https://chatgpt.com/g/g-p-68cfdd0a2f4481919b719ea34cbc603d-my-career/c/README_legacy.md) for the historical Docker implementation._

---

## 🎓 Learning & Research Context

This project serves as a **self-directed research platform** complementing my mechanical-engineering studies.  
It enabled me to integrate **robotics middleware, cloud platforms, and simulation technologies** into one coherent framework.

### Technical Skills Developed

- **Cross-Domain Integration** – Connecting ROS2 robotics with Firebase cloud infrastructure
- **Coordinate Transformations** – GPS (WGS84) ↔ Cartesian (Gazebo)
- **Real-Time System Design** – Managing latency, synchronization, and stability
- **3D Simulation Modeling** – GIS-based modeling in Blender for Gazebo environments
    

### Research Alignment

This work aligns with active research themes such as:

- **Cloud Robotics** – Offloading computation and coordination to cloud backends
- **Fleet Management** – Multi-agent mobility systems for scalable operations
- **Digital Twins** – Simulation-driven validation before physical deployment
    

### Industry Relevance

Industries like **Bosch Mobility**, **AWS RoboMaker**, and **Toyota Research Institute** increasingly seek engineers skilled in:

- Robotics middleware (ROS/ROS2)
- Cloud service integration (AWS IoT, Firebase, GCP)  
- Full-stack system thinking — hardware ↔ software ↔ cloud
    

> _“Effort is not about volume — it’s about designing the structure of learning.”_

---

## 🚀 Future Work

### Current Development Focus

- **Collaborative Expansion** – Preparing the repository for multi-user contributions and modular architecture
- **Real-World Validation** – Testing with physical TurtleBot3 and campus mobility units
- **Performance Refinement** – Reducing latency and improving coordinate precision
- **Documentation & Outreach** – Publishing English guides and demo documentation
    

### Next Development Phase

- **Simulation–Hardware Bridge** – Linking simulated data with real-world sensors
- **Cloud Robotics Exploration** – Extending to AWS IoT Core and Google Cloud Robotics
- **Team Development Framework** – Formalizing contribution workflow for shared development

### Long-Term Direction

Through continuous refinement, I aim to evolve this platform into a **scalable mobility management prototype**—a foundation for intelligent fleet control systems bridging robotics and the cloud.
> _“From code to collaboration — from systems to solutions.”_

---

## 💬 Developer’s Note

Building this system has been a journey of **structured curiosity**.  
Each challenge—from coordinate errors to GPU constraints—reshaped how I approach design and problem-solving.

As a second-year undergraduate who began without formal lab affiliation, I learned that:
- **Initiative beats permission** – Progress starts before the perfect timing
- **Constraints create innovation** – Limited resources foster creativity
- **AI is a multiplier, not a crutch** – Claude and GPT supported structure, but insights came through debugging and iteration

This project is more than a codebase — it is my bridge between learning and contribution, and a reflection of the engineer I am becoming.

---

## 🔗 Links & Resources

- **LinkedIn:** [yugo-dev](https://www.linkedin.com/in/yugo-dev)
- **GitHub Issues:** [Feedback & feature requests](https://github.com/Iruazu/mobility-ros2-firebase/issues)
- **Documentation:** [`README_current.md`](https://chatgpt.com/g/g-p-68cfdd0a2f4481919b719ea34cbc603d-my-career/c/README_current.md) | [`README_legacy.md`](https://chatgpt.com/g/g-p-68cfdd0a2f4481919b719ea34cbc603d-my-career/c/README_legacy.md) | `docs/ARCHITECTURE.md` _(coming soon)_
    

**External References**
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [Firebase Admin SDK (Python)](https://firebase.google.com/docs/admin/setup)
- [TurtleBot3 e-Manual](https://emanual.robotis.com/docs/en/platform/turtlebot3/)
    

---

## 📜 License

Licensed under the **Apache License 2.0** — see [`LICENSE`](https://chatgpt.com/g/g-p-68cfdd0a2f4481919b719ea34cbc603d-my-career/c/LICENSE) for details.  
Chosen for its permissive use, explicit patent grant, and compatibility with both open-source and commercial projects.

---

## 🙏 Acknowledgments

This project was made possible by:
- **ROS2 & Nav2 Community** – open-source robotics middleware  
- **Firebase Team** – real-time cloud infrastructure  
- **OpenStreetMap Contributors** – detailed mapping data 
- **AI Tools (Claude, GPT)** – code and documentation refinement
Special thanks to mentors and peers who encouraged exploration beyond coursework.
---

## ⚠️ Work in Progress

This repository is **actively maintained and evolving**.  
Expect frequent updates, new experiments, and occasional structural changes.

### Current Status

- ✅ Core dispatch system functional (Ubuntu 22.04)
- ✅ Custom Gazebo world integrated
- ✅ Firestore synchronization verified
- 🚧 Nav2 re-integration in progress
- 🚧 Web dashboard development ongoing
- 📋 Hardware testing planned
    

Contributions and feedback are welcome through GitHub Issues.

---

**Built with curiosity, structured with discipline, driven by purpose.**

🤖 **ROS2** • ☁️ **Firebase** • 🎓 **Learning by Building**

_Last Updated: November 2025_  
_Maintainer: Yugo Obana ([@Iruazu](https://github.com/Iruazu))_

---
</div>
