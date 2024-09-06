# BEVNav: Robot Autonomous Navigation Via Spatial-Temporal Contrastive Learning in Bird’s-Eye View

## Abstract

Goal-driven mobile robot navigation in map-less environments requires effective state representations for reliable decision-making. Inspired by the favorable properties of Bird’s- Eye View (BEV) in point clouds for visual perception, this paper introduces a novel navigation approach named BEVNav. It em- ploys deep reinforcement learning to learn BEV representations and enhance decision-making reliability. First, we propose a self-supervised spatial-temporal contrastive learning approach to learn BEV representations. Spatially, two randomly augmented views from a point cloud predict each other, enhancing spatial features. Temporally, we combine the current observation with consecutive frames’ actions to predict future features, establishing the relationship between observation transitions and actions to capture temporal cues. Then, incorporating this spatial-temporal contrastive learning in the Soft Actor-Critic reinforcement learning framework, our BEVNav offers a superior navigation policy. Extensive experiments demonstrate BEVNav’s robustness in environments with dense pedestrians, outperforming state-of-the-art methods across multiple benchmarks. The code will be made publicly available at BEVNav.

#### Main dependencies: 

* [ROS Melodic](http://wiki.ros.org/melodic/Installation)
* [PyTorch](https://pytorch.org/get-started/locally/)
* [Openmmlab](https://openmmlab.com/)

#### Clone the repository:

```shell
$ cd ~
### Clone this repo
$ git clone https://github.com/LanrenzzzZ/BEVNav
```

#### Compile the workspace:

```shell
$ cd ~/BEVNav/catkin_ws
### Compile
$ catkin_make
```

#### Open a terminal and set up sources:

```shell
$ export ROS_HOSTNAME=localhost
$ export ROS_MASTER_URI=http://localhost:11311
$ export ROS_PORT_SIM=11311
$ export GAZEBO_RESOURCE_PATH=~/DMCL/catkin_ws/src/multi_robot_scenario/launch
$ source ~/.bashrc
$ cd ~/DMCL/catkin_ws
$ source devel/setup.bash
$ source install/setup.bash --extend
$ cd ~/BEVNav/SAC
$ conda activate BEVNav
$ python3 train.py
```

#### To kill the training process:

```shell
$ killall -9 rosout roslaunch rosmaster gzserver nodelet robot_state_publisher gzclient python python3
```

## Citation

```bibtex
@article{jiang2024bevnav,
  title={BEVNav: Robot Autonomous Navigation Via Spatial-Temporal Contrastive Learning in Bird's-Eye View},
  author={Jiang, Jiahao and Yang, Yuxiang and Deng, Yingqi and Ma, Chenlong and Zhang, Jing},
  journal={arXiv preprint arXiv:2409.01646},
  year={2024}
}
```
