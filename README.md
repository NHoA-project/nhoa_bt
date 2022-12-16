# nhoa_bt
### NHoA's project BT implementation

## Overview
This is a ROS package developed as part of the [NHoA](https://eurecat.org/en/portfolio-items/nhoa/) project, with the main goal of providing its [Behavior Three (BT)](https://www.behaviortree.dev/) execution orchestrator.\
**Maintainers:** [Óscar Palacín](oscar.palacin@eurecat.org).

## Repository structure

This repository is structured into three packages:

* **nhoa_bt**: BT implementation. It encapsulates all the required functionalities (BT description, action and condition nodes) for the first NHoA's demo.
* **nhoa_msgs**: Custom messages used for communication between BT nodes.
* **nhoa_msgs_dependencies**: Message dependencies used for communication between BT nodes.

## Dependencies

* [BehaviorTree.CPP (v.3.8)](https://github.com/BehaviorTree/BehaviorTree.CPP/tree/v3.8)
* [Groot](https://github.com/BehaviorTree/Groot) (BT GI visualization)
* [play_motion_msgs](https://github.com/pal-robotics/play_motion.git)
- [pal_interaction_msgs](https://github.com/pal-robotics/pal_msgs.git)
- [hri_msgs](https://github.com/ros4hri/hri_msgs.git)
- [nhoa_approach_action](https://github.com/nhoa-project/nhoa_approach_action)
- [nhoa_head_following_action](https://github.com/nhoa-project/nhoa_head_following_action)
- [upo_ssg_msgs](https://github.com/nhoa-project/upo_ssg_msgs)

Tested with Ubuntu 20.04 and ROS Noetic.

## Installation

Clone this repository into a ROS workspace:
```
cd ${WORKSPACE_FOLDER}
git clone https://github.com/NHoA-project/nhoa_bt.git src/nhoa
# Only for EUT.
git clone https://ice.eurecat.org/gitlab/robotics-automation/nhoa src/nhoa
```
If you want to use it in you own machine (ROS noetic). If not, follow __Docker Installation__
Build:
```
catkin build nhoa
source ${WORKSPACE_FOLDER}/devel/setup.bash
```

## Docker Installation 
### From [NHOA's Organization GitHub](https://github.com/orgs/nhoa-project/packages) 
If you do not have access to NHoA's GitHub contact with [Óscar Palacín](oscar.palacin@eurecat.org).

If it is your first time with Docker Package Container Registry, follow this [steps](https://docs.github.com/en/packages/working-with-a-github-packages-registry/working-with-the-container-registry). For more detailed explanation look the following [video](https://www.youtube.com/watch?v=qoMg86QA5P4). 

Pull the Docker Image:
```
docker pull ghcr.io/nhoa-project/noetic-nhoa_bt:latest
```

### From [EURECAT GitLab](https://ice.eurecat.org/gitlab/robotics-automation/nhoa) 

Login in the  Eurecat's Docker image registry (If it is your first time, follow the steps of the [link](https://ice.eurecat.org/gitlab/robotics-automation/robotics-dockers/-/tree/main)):
```
docker login gitlab.local.eurecat.org:5050
```

Pull the images:
```
docker pull gitlab.local.eurecat.org:5050/robotics-automation/nhoa:noetic-nhoa_bt
```


## Run

Run docker compose up from the nhoa_bt folder (it is required to have already pulled the PAL ARI's robot [docker image](https://gitlab.com/pal-robotics/nhoa/dockers)):
```
cd ${WORKSPACE_FOLDER}/src/nhoa/nhoa_bt
docker compose up -d
```

Then, execute in different terminals each docker image's bash:
```
# In the 1st terminal
docker exec -it melodic-ari bash
# In the 2nd terminal
docker exec -it noetic-nhoa_bt bash
# In the 3rd terminal
docker exec -it noetic-nhoa_bt bash
# In the 4rth terminal
docker exec -it noetic-nhoa_bt bash
```
Run the following command to launch all required nodes for the PAL ARI's simulation (1st terminal):
```
source /opt/pal/ferrum/setup.bash
roslaunch ari_20_gazebo ari_gazebo.launch
```
Run the following command to launch the nhoa_approach_action node (2nd terminal):
```
roslaunch nhoa_approach_action nhoa_approach_action.launch
```
Run the following command to launch the nhoa_head_following_action node (3rd terminal):
```
roslaunch nhoa_head_following_action nhoa_head_folllowing_action.launch 
```
Run the following command to launch all required nodes for the NHoA's BT execution (4th terminal):
```
roslaunch nhoa_bt nhoa_bt_demo.launch
```
## Additional
If you want to export the GI to your device, you will need to execute the following steps in a new terminal:

### Option 1
Create the "docker" folder at home.
```
mkdir ~/docker
```
Add the ".xauth_docker" folder in the docker folder.
```
cd docker 
cp -r ${WORKSPACE_FOLDER}/src/nhoa/nhoa_bt/.xauth_docker .
```

Modify the "$HOME_PATH" of the ".xauth_docker/config/set_xauth_for_docker.sh" file for your own
```
nano ~/docker/.xauth_docker/config/set_xauth_for_docker.sh 
```
Run sudo bash every time the PC is rebooted.
```
sudo bash ~/docker/.xauth_docker/config/set_xauth_for_docker.sh 
```

### Option 2

Modify the "$HOME_PATH" of the ".xauth_docker/config/set_xauth_for_docker.sh" file for your own
```
nano .xauth_docker/config/set_xauth_for_docker.sh 
```
Modify the "$WORKSPACE" of the "docker-compose.yaml" file for your own
```
nano .xauth_docker/config/set_xauth_for_docker.sh 
```
Run sudo bash every time the PC is rebooted.
```
sudo bash .xauth_docker/config/set_xauth_for_docker.sh 
```

