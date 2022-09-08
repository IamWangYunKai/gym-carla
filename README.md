#  gym-carla

Fork from [https://github.com/cjy1992/gym-carla](https://github.com/cjy1992/gym-carla)

#### New Features

- [x] Change waypoints color according to the traffic light
- [x] Boost control frequency from 10Hz to 20Hz, and update lidar's params
- [x] Add AI driver
- [x] Automatic data collection
- [x] Draw trajectory
- [ ] Multi-agent control

## Install

* Install [CARLA 0.9.6](https://github.com/carla-simulator/carla/releases)

* Install this repo:

```bash
pip install -r requirements.txt
pip install -e .
```

## Run

* Start CARLA:
Go to CARLA path, and run
```bash
DISPLAY= ./CarlaUE4.sh -opengl
```

## Description
* We provide a dictionary observation including front view camera (obs['camera']), birdeye view lidar point cloud (obs['lidar']) and birdeye view semantic representation (obs['birdeye']).
We also provide a state vector observation (obs['state']) which is composed of lateral distance and heading error between the ego vehicle to the target lane center line (in meter and rad), ego vehicle's speed (in meters per second), and and indicator of whether there is a front vehicle within a safety margin.

* The termination condition is either the ego vehicle collides, runs out of lane, reaches a destination, or reaches the maximum episode timesteps. Users may modify function _terminal in carla_env.py to enable customized termination condition.

* The reward is a weighted combination of longitudinal speed and penalties for collision, exceeding maximum speed, out of lane, large steering and large lateral accleration. Users may modify function _get_reward in carla_env.py to enable customized reward function.

## LICENSE
MIT LICENSE