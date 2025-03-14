# FUSION_FR3
<img src="assets/fusion_fr3.png" width=300>

## TODOs
- implement controllers

  | controller | implementation status |
  | :---: | :---: |
  | template controller | :x: |
  | zero gravity | :white_check_mark: |
  | cartesian velocity | :white_check_mark: |
  | cartesian pose | :white_check_mark: |
  | cartesian force | :x: |
  | cartesian impedance + velocity | :x: |
  | cartesian admittance + velocity | :x: |

- implement controller switch service
- implement ROS parameter server for controllers
  - set Cartesian impedance
  - set pose goal tolerance
  - set pose control speed
- implement FK & IK interface
- implement DESK interface
  - start/stop FCI
  - lock/unlock joints
  - recover
  - pilot button events
- (?) simulated robot interface

## Installation

### 0. dependencies
```FUSION_FR3``` is developed and tested on Ubuntu 20.04 and ROS Noetic.
```libfranka``` is necessary. The installation guide can be found [here](https://github.com/frankaemika/libfranka/blob/main/README.md). Real-time kernel is optional.

### 1. install panda-py
```FUSION_FR3``` is based on [```panda-py```](https://github.com/JeanElsner/panda-py), whose version needs to match ```libfranka``` version. 
For ```libfranka``` 0.13.3, the corresponding ```panda-py``` release can be found under ```/assets``` folder. Unzip the release folder, then
```shell
pip install panda_python-0.8.1+libfranka.0.13.0-*python version*-manylinux_2_17_x86_64.manylinux2014_x86_64.whl
```

### 2. install other python packages

```shell
pip3 install numpy
pip3 install spatialmath
pip3 install qpsolvers
```

### 3. install FUSION_FR3
```shell
cd ~/catkin_ws/src
git clone https://github.com/MXHsj/fusion_fr3.git
cd ..
```
if building the catkin workspace for the first time:
```shell
catkin_make -DCMAKE_BUILD_TYPE=Release -DFranka_DIR:PATH=/path/to/libfranka/build
source devel/setup.sh
```
otherwise:
```shell
catkin_make
source devel/setup.sh
```

## Usage

### 0. robot bring-up
```shell
roslaunch fusion_fr3 robot_bringup.launch
```

### 1. ROS Topics
| name | description |
| :---: | :---: |
| ```fr3/controller/Cartesian/velocity``` | send Cartesian velocity command w.r.t ee frame  |
| ```fr3/controller/joint/velocity``` | send joint velocity command |
| ```fr3/state/F_ext``` | external wrench w.r.t ee frame  |
| ```fr3/state/O_T_EE``` | ee pose w.r.t base frame |
| ```fr3/state/q``` | joint angles |
| ```fr3/state/body_jacobian``` | body Jacobian |
| ```fr3/state/zero_jacobian``` | zero Jacobian |

### 2. ROS Actions
| name | description |
| :---: | :---: |
| ```fr3/action/move_to_pose``` | move ee to a pose goal |

### 3. ROS Services
| name | description |
| :---: | :---: |
| ```fr3/service/move_to_start``` | move to start configuration |