# External Forces Resilient Planner

A local planner considering external forces. The code will be released after the acceptance of this paper

**Video Links**  [youtube](https://youtu.be/nSKbzAM0v18) or [bilibili](https://www.bilibili.com/video/BV1eX4y137vn/) (for Mainland China)

**Related Paper** [arxiv](https://arxiv.org/pdf/2103.11178.pdf)

# 1. Prerequisites

## 1.1 Simulators

we use [RotorS](https://github.com/ethz-asl/rotors_simulator) as our simulation environment.

## 1.2 Solver

[Forces Pro](https://www.embotech.com/products/forcespro/overview/) is applied to solve nonlinear Model Predictive Control. You can directly use the compiled version in this repo. Or if you want to change any settiings of the solver, you should request an academic license [here](https://www.embotech.com/products/forcespro/licensing/) and replace the previous lib. 

## 1.3 Corridor Generator

we use [Galaxy](https://github.com/ZJU-FAST-Lab/Galaxy) to generate corridor directly on point clouds. The code will be release after publication of this paper. 


## 1.4 External Force Estimator

we use [VID-Fusion](https://github.com/ZJU-FAST-Lab/VID-Fusion) to get external force.


