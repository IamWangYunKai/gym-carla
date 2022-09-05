#!/usr/bin/env python
CARLA_PATH = '/home/wang/CARLA_0.9.6/'

import sys
sys.path.append(CARLA_PATH + 'PythonAPI/carla')
sys.path.append(CARLA_PATH + 'PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')
import carla
from agents.navigation.basic_agent import BasicAgent

import cv2
import gym
import gym_carla

def main():
    # parameters for the gym_carla environment
    params = {
        'number_of_vehicles': 100,  #100,
        'number_of_walkers': 100,
        'display_size': 256,  # screen size of bird-eye render
        'max_past_step': 10,  #,1,  # the number of past steps to draw
        'dt': 0.05, #0.1,  # time interval between two frames
        'discrete': False,  # whether to use discrete control space
        'discrete_acc': [-3.0, 0.0, 3.0],  # discrete value of accelerations
        'discrete_steer': [-0.2, 0.0, 0.2],  # discrete value of steering angles
        'continuous_accel_range': [-3.0, 3.0],  # continuous acceleration range
        'continuous_steer_range': [-0.3, 0.3],  # continuous steering angle range
        'ego_vehicle_filter': 'vehicle.bmw.grandtourer',  #'vehicle.lincoln*',  # filter for defining ego vehicle
        'port': 2000,  # connection port
        'town': 'Town01',  # which town to simulate
        'task_mode': 'random',  # mode of the task, [random, roundabout (only for Town03)]
        'max_time_episode': 3000,  # maximum timesteps per episode
        'max_waypt': 12,  # maximum number of waypoints
        'obs_range': 32,  # observation range (meter)
        'lidar_bin': 0.125,  # bin size of lidar sensor (meter)
        'd_behind': 12,  # distance behind the ego vehicle (meter)
        'out_lane_thres': 5.0,  #2.0,  # threshold for out of lane
        'desired_speed': 8,  # desired speed (m/s)
        'max_ego_spawn_times': 200,  # maximum times to spawn ego vehicle
        'display_route': True,  # whether to render the desired route
        'pixor_size': 64,  # size of the pixor labels
        'pixor': False,  # whether to output PIXOR observation
    }

    # Set gym-carla environment
    env = gym.make('carla-v0', params=params)
    obs = env.reset()

    agent = BasicAgent(env.ego, target_speed=params['desired_speed'] * 3.6)

    control = carla.VehicleControl(throttle=float(0), steer=float(0), brake=float(0))
    obs, r, done, info = env.raw_step(control)
    agent.set_destination(info['waypoints'][5])
    control = agent.run_step()

    while True:
        # action = [2.0, 0.0]
        control = agent.run_step()
        obs, r, done, info = env.raw_step(control)
        agent.set_destination(info['waypoints'][5])

        # obs,r,done,info = env.step(action)
        # cv2.imshow('img', obs['birdeye'])
        # cv2.waitKey(5)
        if done:
          print('reset !')
          obs = env.reset()
          agent = BasicAgent(env.ego, target_speed=params['desired_speed'] * 3.6)
          control = carla.VehicleControl(throttle=float(0), steer=float(0), brake=float(0))
          obs, r, done, info = env.raw_step(control)
          agent.set_destination(info['waypoints'][5])
          control = agent.run_step()


if __name__ == '__main__':
    main()
