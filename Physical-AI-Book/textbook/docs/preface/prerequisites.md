---
title: "Prerequisites"
slug: prerequisites
sidebar_label: "Prerequisites"
toc: true
description: "Required knowledge, software, and hardware for completing the Physical-AI textbook curriculum"
---

# Prerequisites

This page outlines the knowledge, software, and hardware required to successfully complete this textbook.

## Knowledge Prerequisites

### Required Background

You should be comfortable with:

#### 1. Programming (Python or C++)

**Python** (Primary language for this textbook):
- Functions, classes, modules
- Lists, dictionaries, loops
- File I/O and exception handling
- Using `pip` and virtual environments

**C++** (Optional, for performance-critical ROS 2 nodes):
- Basic syntax and object-oriented programming
- Pointers and references
- Building with CMake

**Self-assessment**: Can you write a Python class that reads a CSV file and computes statistics? If yes, you're ready.

**Recommended prep** (if needed):
- [Python for Everybody](https://www.py4e.com/) (Dr. Chuck, free)
- [Automate the Boring Stuff](https://automatetheboringstuff.com/) (Al Sweigart, free)

#### 2. Linear Algebra

You'll need familiarity with:
- Vectors and matrices
- Dot products and cross products
- Coordinate transformations (rotation matrices)
- Basic quaternions (we'll review these in Module 1)

**Self-assessment**: Can you multiply two 3x3 matrices and explain what a rotation matrix does? If yes, you're ready.

**Recommended prep**:
- [3Blue1Brown's Essence of Linear Algebra](https://www.youtube.com/playlist?list=PLZHQObOWTQDPD3MizzM2xVFitgF8hE_ab) (YouTube, visual explanations)
- Khan Academy Linear Algebra (free)

#### 3. Calculus (Basic)

You'll see:
- Derivatives (for understanding velocity/acceleration)
- Integrals (for motion planning)

**No proof-heavy math required**—just intuition about rates of change.

**Self-assessment**: Do you understand what a derivative represents physically? If yes, you're ready.

#### 4. Command-Line Literacy

You should be able to:
- Navigate directories (`cd`, `ls`, `pwd`)
- Run scripts and binaries
- Install software via package managers (`apt`, `pip`, `npm`)
- Use a text editor (VS Code, Vim, etc.)

**Recommended prep**: [The Missing Semester of Your CS Education](https://missing.csail.mit.edu/) (MIT, free)

### Nice-to-Have (Not Required)

- **ROS 1 experience**: Helpful but not necessary (we teach ROS 2 from scratch)
- **Control theory**: We introduce what's needed; deeper background helps with Module 3-4
- **Computer vision**: We cover basics; OpenCV experience is a plus
- **Machine learning**: Familiarity with neural networks helps in Module 4

## Software Prerequisites

### Operating System

**Required**: Ubuntu 22.04 LTS (Jammy Jellyfish)

**Why Ubuntu?** ROS 2 Humble (the version we use) is officially supported on Ubuntu 22.04. Other distros *may* work but will cause dependency issues.

**Options**:
1. **Native install** (Recommended): Dual-boot or dedicated machine
2. **Virtual machine**: Use VirtualBox or VMware (slower, especially for simulation)
3. **Docker**: Advanced users can use ROS 2 Docker containers
4. **Cloud**: AWS EC2, GitHub Codespaces (guide in Appendix)

**Not supported**:
- ❌ Windows (even with WSL2—GPU passthrough is problematic)
- ❌ macOS (no official ROS 2 support)

If you're on Windows/Mac, use a VM or cloud instance.

### Core Software Stack

#### Module 1 Requirements

| Software | Version | Installation | Purpose |
|----------|---------|--------------|---------|
| **ROS 2 Humble** | Desktop Full | [ros.org/humble](https://docs.ros.org/en/humble/Installation.html) | Robot middleware |
| **Python** | 3.10+ | `sudo apt install python3-pip` | Scripting |
| **VS Code** | Latest | [code.visualstudio.com](https://code.visualstudio.com/) | Code editor |
| **Git** | 2.34+ | `sudo apt install git` | Version control |

**Installation time**: ~1 hour

**Verification**:
```bash
source /opt/ros/humble/setup.bash
ros2 --version
# Expected output: ros2 cli version 0.18.x
```

#### Module 2 Requirements (Add to Module 1)

| Software | Version | Installation | Purpose |
|----------|---------|--------------|---------|
| **Gazebo Fortress** | 6.x | [gazebosim.org](https://gazebosim.org/docs/fortress/install_ubuntu) | Physics simulation |
| **Unity** | 2022.3 LTS | [unity.com](https://unity.com/download) | Rendering/visualization |
| **Blender** | 3.6+ | `sudo snap install blender --classic` | 3D modeling (optional) |

**Installation time**: ~2 hours

#### Module 3 Requirements (Add to Module 2)

| Software | Version | Installation | Purpose |
|----------|---------|--------------|---------|
| **NVIDIA Isaac Sim** | 2023.1.1+ | [NVIDIA Omniverse](https://www.nvidia.com/en-us/omniverse/) | GPU-accelerated sim |
| **Isaac ROS** | Latest | [NVIDIA Isaac ROS](https://nvidia-isaac-ros.github.io/getting_started/index.html) | Perception packages |
| **Docker** | 24.0+ | [docker.com](https://docs.docker.com/engine/install/ubuntu/) | Containerization |
| **CUDA** | 12.1+ | Auto-installed with Isaac Sim | GPU compute |

**Installation time**: ~3 hours (large downloads)

**Requires**: NVIDIA GPU (see Hardware Requirements)

#### Module 4 Requirements (Add to Module 3)

| Software | Version | Installation | Purpose |
|----------|---------|--------------|---------|
| **PyTorch** | 2.0+ | [pytorch.org](https://pytorch.org/get-started/locally/) | Deep learning framework |
| **Whisper** | Latest | `pip install openai-whisper` | Speech recognition |
| **Transformers** | 4.30+ | `pip install transformers` | LLM integration |

**Installation time**: ~30 minutes

### Dependency Management

**Use virtual environments** to avoid version conflicts:

```bash
# Create environment for this textbook
python3 -m venv ~/robotics-env

# Activate it
source ~/robotics-env/bin/activate

# Install Python packages
pip install numpy matplotlib jupyter
```

**Docker alternative** (advanced):
We provide a `Dockerfile` with all dependencies pre-installed:

```bash
docker pull physicalai/textbook:latest
docker run -it --gpus all physicalai/textbook:latest
```

## Hardware Requirements

### Minimum Specifications

**For Modules 1-2** (no GPU required):

| Component | Minimum | Notes |
|-----------|---------|-------|
| **CPU** | 4 cores, 2.5 GHz | Intel i5-8400 or AMD Ryzen 5 3600 |
| **RAM** | 16 GB | 8 GB possible but will struggle with Gazebo |
| **Storage** | 100 GB SSD | HDD will slow compilation significantly |
| **GPU** | Integrated graphics | For basic visualization only |

**Budget**: ~$600 used laptop or desktop

### Recommended Specifications

**For Modules 1-4** (includes GPU for Isaac Sim):

| Component | Recommended | Notes |
|-----------|-------------|-------|
| **CPU** | 8+ cores, 3.0+ GHz | Intel i7-12700 or AMD Ryzen 7 5800X |
| **RAM** | 32 GB | Isaac Sim uses 10-15 GB under load |
| **Storage** | 256 GB NVMe SSD | Fast I/O crucial for simulation |
| **GPU** | NVIDIA RTX 3060+ | 12 GB VRAM minimum for Isaac Sim |

**Budget**: ~$1,500 desktop build

**GPU importance**: Modules 3-4 **require** an NVIDIA GPU with:
- RTX series (2000, 3000, or 4000)
- 12+ GB VRAM
- CUDA compute capability 7.0+

**Checking your GPU**:
```bash
nvidia-smi
# Look for "Tesla", "RTX", or "Quadro" in the name
```

**No GPU?** Use cloud alternatives:
- **AWS EC2**: g5.xlarge instances (~$1.50/hour)
- **Paperspace**: RTX 4000 machines (~$0.50/hour)
- **Google Colab**: Free tier has T4 GPUs (limited runtime)

### Cloud Setup Guide

**AWS EC2 (Recommended for Modules 3-4)**:

1. Launch a `g5.xlarge` instance (Ubuntu 22.04 Deep Learning AMI)
2. Install ROS 2 Humble:
   ```bash
   sudo apt update
   sudo apt install ros-humble-desktop-full
   ```
3. Install Isaac Sim via Omniverse launcher
4. Connect via SSH with X11 forwarding or use VNC

**Cost estimate**: ~$100-150 for the entire course if you use cloud only for Modules 3-4.

**Detailed guide**: See Appendix - Cloud Setup Tutorial

## Development Tools

### Recommended VS Code Extensions

Install these for the best experience:

```bash
code --install-extension ms-python.python
code --install-extension ms-vscode.cpptools
code --install-extension ms-iot.vscode-ros
code --install-extension streetsidesoftware.code-spell-checker
```

**Extensions**:
- **ROS**: Syntax highlighting for `.launch`, `.urdf` files
- **Python**: Linting, debugging
- **C/C++**: IntelliSense for ROS 2 packages
- **Spell Checker**: Catches typos in documentation

### Terminal Multiplexer (Optional but Useful)

ROS 2 workflows involve running many terminal windows. Use `tmux` or `terminator`:

```bash
sudo apt install tmux
```

**Why?** You'll often run:
- Terminal 1: ROS 2 core services
- Terminal 2: Gazebo simulator
- Terminal 3: Your robot node
- Terminal 4: Debugging tools

`tmux` lets you manage these in one window with splits.

## Verification Checklist

Before starting Module 1, verify your setup:

### Software Checks

Run these commands and confirm output:

```bash
# 1. Ubuntu version
lsb_release -a
# Expected: Ubuntu 22.04.x LTS

# 2. ROS 2 Humble
source /opt/ros/humble/setup.bash
ros2 --version
# Expected: ros2 cli version 0.18.x

# 3. Python version
python3 --version
# Expected: Python 3.10.x or 3.11.x

# 4. Gazebo (Module 2)
ign gazebo --version
# Expected: Gazebo Fortress 6.x

# 5. NVIDIA GPU (Module 3-4)
nvidia-smi
# Expected: Driver version 525+ and CUDA 12.x
```

### Test Launch

Run a sample ROS 2 node:

```bash
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker
```

**Expected output**:
```
[INFO] [timestamp]: Publishing: 'Hello World: 1'
[INFO] [timestamp]: Publishing: 'Hello World: 2'
...
```

Press `Ctrl+C` to stop.

### Troubleshooting Common Issues

**Issue**: `ros2: command not found`
**Fix**: Add to `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
```

**Issue**: Gazebo crashes with "libGL error"
**Fix**: Install missing OpenGL libraries:
```bash
sudo apt install mesa-utils libgl1-mesa-glx
```

**Issue**: Isaac Sim won't launch
**Fix**: Verify GPU drivers:
```bash
sudo ubuntu-drivers autoinstall
sudo reboot
```

**Still stuck?** See Appendix - Troubleshooting FAQ or ask on Discord.

## Physical Robot Hardware (Optional)

This textbook can be completed **entirely in simulation**. No physical robot is required.

**If you want to deploy to real hardware** (optional, advanced):

Compatible robots:
- **TurtleBot 3** (~$800): Beginner-friendly, well-documented
- **Clearpath Jackal** (~$15,000): Industrial-grade UGV
- **PAL Robotics TALOS** (~$100,000+): Full humanoid (university labs only)

**Recommendation**: Complete the textbook in simulation first, then transfer skills to hardware as a capstone project.

## Time Commitment

**Estimated setup time**:
- Module 1 software: 2-3 hours
- Module 2 additions: 2 hours
- Module 3 additions (with GPU): 4-6 hours
- Module 4 additions: 1 hour

**Total**: ~10-14 hours one-time setup

**Learning time per module**: 30-40 hours (reading + exercises)

**Full textbook**: ~150-180 hours (one semester at 10 hours/week)

## Still Not Sure If You're Ready?

Take this **self-assessment quiz**:

1. Can you write a Python function that takes a list and returns the average?
2. Do you know what a matrix is and how to multiply two matrices?
3. Can you navigate to a directory in the terminal and list its files?
4. Do you have access to a computer running Ubuntu 22.04 (native, VM, or cloud)?

**If you answered YES to all four**: You're ready to start Module 1.

**If you answered NO to any**: Review the recommended prep materials above, then come back.

---

## Next Steps

✅ Prerequisites met? Read [Ethics and Safety](./ethics-and-safety.md) before proceeding.

Then start **[Module 1: Foundations (ROS 2)](../foundations-ros2/index.md)**.
