# External Forces Resilient Planner

External Forces Resilient Safe Motion Planning for Quadrotor
 

##  About

 External Forces Resilient Planner is a systematic framework to achieve robust local planning, which accounts for the influence of extreme external disturbance.


<p align="center">
  <img src="docs/1.gif" width = "390" height = "219.7"/>
  <img src="docs/2.gif" width = "390" height = "219.7"/>
  <img src="docs/3.gif" width = "390" height = "219.7"/>
  <img src="docs/4.gif" width = "390" height = "219.7"/>
</p>


__Authors__: [Yuwei WU](https://www.seas.upenn.edu/~yuweiwu/)  and [Fei GAO](https://ustfei.com/) from the [ZJU Fast Lab](http://www.zju-fast.com).

__Video Links__:  [youtube](https://youtu.be/nSKbzAM0v18) or [bilibili](https://www.bilibili.com/video/BV1eX4y137vn/) (for Mainland China)

__Related Paper__: Y. Wu, Z. Ding, C. Xu and F. Gao, "External Forces Resilient Safe Motion Planning for Quadrotor," in IEEE Robotics and Automation Letters, vol. 6, no. 4, pp. 8506-8513. [arxiv Preprint](https://arxiv.org/pdf/2103.11178.pdf), [IEEE Xplore](https://ieeexplore.ieee.org/abstract/document/9531427/)

__Architecture__:
<p align="center">
  <img src="docs/system.png" width = "432" height = "324"/>
</p>


If this repo helps your research, please cite our paper at:
```bibtex
@ARTICLE{9531427,
  author={Wu, Yuwei and Ding, Ziming and Xu, Chao and Gao, Fei},
  journal={IEEE Robotics and Automation Letters}, 
  title={External Forces Resilient Safe Motion Planning for Quadrotor}, 
  year={2021},
  volume={6},
  number={4},
  pages={8506-8513},
  doi={10.1109/LRA.2021.3110316}}
```

##  Acknowledgements

- ***Simulators*** : we use [RotorS](https://github.com/ethz-asl/rotors_simulator) as our simulation environment.
- ***NMPC Solver*** ：[Forces Pro](https://www.embotech.com/products/forcespro/overview/) is applied to generate c library to solve nonlinear Model Predictive Control automatically. Thanks [mrca-mav](https://github.com/tud-amr/mrca-mav) as the example about how to use Forces Pro for MPC.
- ***External Force Estimator***: we use [VID-Fusion](https://github.com/ZJU-FAST-Lab/VID-Fusion) to get external force.
- ***Corridor Generator*** ：We use [Galaxy](https://github.com/ZJU-FAST-Lab/Galaxy) by Xingguang to generate corridor directly on point clouds in our paper and the code will be released after the publication of this paper. Now we use [MRSL Decomp Util ROS](https://github.com/sikang/DecompROS) by Sikang to replace this part. 
- ***Front-end Framework***: The occupancy map is based on [TGK-Planner](https://github.com/ZJU-FAST-Lab/TGK-Planner), and the front-end path searching is revised on the framework of [Fast-Planner](https://github.com/HKUST-Aerial-Robotics/Fast-Planner).

# Run The Simulation
The repo has been tested on 18.04 with ros-desktop-full installation.

## 1. Prerequisites

#### [RotorS](https://github.com/yuwei-wu/rotors_simulator)

Follow the instructions of the revised version and create a new catkin workspace at ``${YOUR_ROTORS_WORKSPACE_PATH}`` for Rotor Simulator (use catkin build). We make some revisions to this repository to fit our planner, and you would better clone the repo [here](https://github.com/yuwei-wu/rotors_simulator) and check more details.

#### [Forces Pro](https://www.embotech.com/products/forcespro/overview/) and Matlab

Firstly, request an academic license [here](https://www.embotech.com/products/forcespro/licensing/). 
You can directly use the compiled version in this repo for testing. And if you want to change the settings of the NMPC solver, you need to use Matlab on ubuntu to regenerate it automatically and replace the previously generated code.

After you build on ROS, you can find the folder ``src/resilient_planner/plan_manage/matlab_code``. In the Matlab terminal, run ``generate_solver.m``, then re-build in the workspace, and you can get the newly generated code in the folder ``src/resilient_planner/plan_manage/solver`` for usage.

## 2. Build on ROS

Clone the repo:
```
git clone https://github.com/ZJU-FAST-Lab/forces_resilient_planner
cd forces_resilient_planner/src/ThirdParty
git clone https://github.com/ZJU-FAST-Lab/VID-Fusion
cd ../..
catkin_make
```

## 3. Run

In ``${YOUR_ROTORS_WORKSPACE_PATH}``, set up the environment and launch the planner:
```
source devel/setup.bash
roslaunch rotors_gazebo fast_with_vi_sensor.launch
```

Then you will see the default map in the gazebo GUI while the quadrotor is on the ground.

Second, set up VID-Fusion in our workspace, 

```
source devel/setup.bash
roslaunch vid_estimator vid_sim.launch
```

Then you can use the keyboard to move the quadrotor around for VID-Fusion initializations. For details about the estimator setup, please carefully read the documents of VID-Fusion. 

<p align="center">
  <img src="docs/keyboard.png" width = "793.5" height = "446.5"/>
</p>


After finishing the setup process, use the keyboard to make the quadrotor hovering (around z = 1.0)


In directory ``~/forces_resilient_planner``, open a new terminal and launch the resilient planner:
```
source devel/setup.bash && roslaunch resilient_planner rotors_sim.launch
```

If everything goes well, you should be able to navigate the drone by 2D-Nav Goal.

## 4. Add the External Force

The research of the wind field is out of the scope of our paper, we only provide several simple methods to add external forces. 
You can add the external force by:

- keyboard to add manual random external force in different dimensions (see the keyboard image)
- disturb manager to encode constant external force with some conditions (eg: vary with time, position)
- the customed wind field, follow the instructions [here](https://github.com/ethz-asl/rotors_simulator/wiki/Adding-a-custom-wind-field-to-your-world)

# Licence
The source code is released under [GPLv3](http://www.gnu.org/licenses/) license.

# Maintaince

For any technical issues, please contact Yuwei Wu (yuweiwu@seas.upenn.edu, yuweiwu20001@outlook.com) or Fei GAO (fgaoaa@zju.edu.cn).

For commercial inquiries, please contact Fei GAO (fgaoaa@zju.edu.cn).
