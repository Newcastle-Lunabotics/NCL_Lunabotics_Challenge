# Development Environment Setup

Setup guide for NCL Lunabotics development on macOS using OrbStack and ROS2 Humble.

## Prerequisites

- macOS 11.0+
- GitHub Account
- Internet Connection

## Setup Steps

### 1. Install Orbstack & Create VM
```bash
# Install homebrew (if needed)
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"

# Install OrbStack
brew install orbstack
open -a Orbstack

# Create Ubuntu 22.04 VM
orb create ubuntu:22.04 lunabotics
orb shell lunabotics
```

Your prompt should change to `ubuntu@lunabotics:~$`

### 2. Install ROS2 Humble
```bash
# Update and install prerequisites
sudo apt update && sudo apt upgrade -y
sudo apt install software-properties-common curl -y
sudo add-apt-repository universe -y

# Add ROS2 repository
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS2 (takes 5-10 minutes)
sudo apt update
sudo apt install ros-humble-desktop -y

# Install build tools and Git
sudo apt install python3-colcon-common-extensions python3-pip git -y

# Auto-load ROS2
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 3. Configure Git & Clone Repository
```bash
# Set Git credentials
git config --global user.name "Your Name"
git config --global user.email "your.email@example.com"

# Clone and build
cd ~
git clone https://github.com/SahasT23/NCL_Lunabotics_Challenge.git
cd NCL_Lunabotics_Challenge/ros2_ws
colcon build
source install/setup.bash

# Auto-load workspace
echo "source ~/NCL_Lunabotics_Challenge/ros2_ws/install/setup.bash" >> ~/.bashrc
```

### 4. Verify Setup

**Terminal 1:**
```bash
orb shell lunabotics
cd ~/NCL_Lunabotics_Challenge/ros2_ws
ros2 run ncl_example_pubsub listener
```

**Terminal 2:**
```bash
orb shell lunabotics
cd ~/NCL_Lunabotics_Challenge/ros2_ws
ros2 run ncl_example_pubsub talker
```

If messages flow between terminals, setup is complete! ✅ Press `Ctrl+C` to stop.

## Daily Workflow
```bash
# Enter VM
orb shell lunabotics
cd ~/NCL_Lunabotics_Challenge/ros2_ws

# After making changes, rebuild
colcon build
source install/setup.bash

# Build specific package only
colcon build --packages-select package_name
```

## Troubleshooting

| Issue | Solution |
|-------|----------|
| OrbStack not found | `brew install orbstack && open -a OrbStack` |
| VM won't start | `orb list` then `orb delete lunabotics` and recreate |
| Disk full | `exit` VM, then `orb config lunabotics --disk 30GB` |
| ROS2 commands not found | `source /opt/ros/humble/setup.bash` |

## System Info

| Component | Version |
|-----------|---------|
| Ubuntu | 22.04 LTS |
| ROS2 | Humble |
| Python | 3.10 |
| Min RAM | 8GB (16GB recommended) |
| Min Disk | 20GB (30GB recommended) |

## Resources

- [ROS2 Humble Docs](https://docs.ros.org/en/humble/)
- [OrbStack Docs](https://docs.orbstack.dev/)
- [Colcon Tutorial](https://colcon.readthedocs.io/)

---

**Maintained by:** Benjamin Matapo | **Updated:** January 2025