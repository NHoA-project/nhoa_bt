# nhoa_bt
### NHoA's project BT implementation

## Overview
This is a ROS package developed as part of the [NHoA](https://eurecat.org/en/portfolio-items/nhoa/) project, with the main goal of providing the [Behavior Three (BT)](https://www.behaviortree.dev/) execution orchestrator.\
**Maintainers:** [Óscar Palacín](oscar.palacin@eurecat.org).

## Repository structure

This repository is structured into four packages:

* **nhoa_bt**: Metapackage.

## Dependencies

* [BehaviorTree.CPP](https://github.com/BehaviorTree/BehaviorTree.CPP)
* [Groot](https://github.com/BehaviorTree/Groot) (for behavior visualization)

Tested with Ubuntu 20.04 and ROS Noetic.

## Installation

Clone this repository into a ROS workspace:
```
mkdir ${WORKSPACE_FOLDER}/src -p
cd ${WORKSPACE_FOLDER}
git clone https://ice.eurecat.org/gitlab/robotics-automation/eut_agro_navigation src/eut_agro_navigation

```

Install [vcstool](http://wiki.ros.org/vcstool) and download required dependencies:
```
sudo apt install python3-vcstool
cd src/eut_agro_navigation
vcs import < dependencies.rosinstall
cd ../..
```

Build:
```
catkin build eut_agro_navigation
source ${WORKSPACE_FOLDER}/devel/setup.bash
```

## Run

Run the following command to launch all required nodes for the simulation:

```
roslaunch eut_agro_simulation eut_agro_simulation_husky.launch
```