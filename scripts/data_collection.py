#!/usr/bin/env python
CARLA_PATH = '/home/wang/CARLA_0.9.6/'

import sys
sys.path.append(CARLA_PATH + 'PythonAPI/carla')
sys.path.append(CARLA_PATH + 'PythonAPI/carla/dist/carla-0.9.6-py3.5-linux-x86_64.egg')
import carla
from agents.navigation.basic_agent import BasicAgent

import os
import cv2
import gym
import gym_carla

save_path = '/media/wang/CYX/CARLA_HDMap/town01/'
dataset_index = 0
file_mgr = {}

def mkdir(path):
    os.makedirs(path, exist_ok=True)

def bgr2rgb(img):
    return cv2.cvtColor(img, cv2.COLOR_BGR2RGB)

def create_dataset():
    global save_path, dataset_index, file_mgr
    path = save_path + str(dataset_index) + '/'
    mkdir(path)
    mkdir(path + 'camera/')
    mkdir(path + 'lidar/')
    mkdir(path + 'birdeye/')
    mkdir(path + 'state/')

    file_mgr['pos'] = open(path+'state/pos.txt', 'w+')
    file_mgr['vel'] = open(path+'state/vel.txt', 'w+')
    file_mgr['acc'] = open(path+'state/acc.txt', 'w+')
    file_mgr['angular_vel'] = open(path+'state/angular_vel.txt', 'w+')
    file_mgr['state'] = open(path+'state/state.txt', 'w+')

def close_dataset():
    global save_path, dataset_index, file_mgr

    file_mgr['pos'].close()
    file_mgr['vel'].close()
    file_mgr['acc'].close()
    file_mgr['angular_vel'].close()
    file_mgr['state'].close()

    dataset_index += 1

def save_data(index, obs, vehicle):
    global save_path, dataset_index, file_mgr
    path = save_path + str(dataset_index) + '/'

    pos = vehicle.get_transform()
    vel = vehicle.get_velocity()
    acceleration = vehicle.get_acceleration()
    angular_velocity = vehicle.get_angular_velocity()

    file_mgr['pos'].write(str(index)+'\t'+
        str(pos.location.x)+'\t'+
        str(pos.location.y)+'\t'+
        str(pos.location.z)+'\t'+
        str(pos.rotation.pitch)+'\t'+
        str(pos.rotation.yaw)+'\t'+
        str(pos.rotation.roll)+'\t'+'\n')
    file_mgr['vel'].write(str(index)+'\t'+
        str(vel.x)+'\t'+
        str(vel.y)+'\t'+
        str(vel.z)+'\t'+'\n')
    file_mgr['acc'].write(str(index)+'\t'+
        str(acceleration.x)+'\t'+
        str(acceleration.y)+'\t'+
        str(acceleration.z)+'\t'+'\n')
    file_mgr['angular_vel'].write(str(index)+'\t'+
        str(angular_velocity.x)+'\t'+
        str(angular_velocity.y)+'\t'+
        str(angular_velocity.z)+'\t'+'\n')
    file_mgr['state'].write(str(index)+'\t'+
        str(obs['state'][0])+'\t'+ # lateral distance (m)
        str(obs['state'][1])+'\t'+ # heading error (rad)
        str(obs['state'][2])+'\t'+ # ego vehicle's speed (m/s)
        str(obs['state'][3])+'\t'+'\n') # whether there is a front vehicle within a safety margin (0/1)

    cv2.imwrite(path + 'birdeye/' + str(index) + '.png', bgr2rgb(obs['birdeye']))
    cv2.imwrite(path + 'lidar/' + str(index) + '.png', bgr2rgb(obs['lidar']))
    cv2.imwrite(path + 'camera/' + str(index) + '.png', bgr2rgb(obs['camera']))


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
        'max_time_episode': 12000,  # maximum timesteps per episode
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
    env.world.set_weather(carla.WeatherParameters.ClearNoon)
    obs = env.reset()

    agent = BasicAgent(env.ego, target_speed=params['desired_speed'] * 3.6)

    control = carla.VehicleControl(throttle=float(0), steer=float(0), brake=float(0))
    obs, r, done, info = env.raw_step(control)
    agent.set_destination(info['waypoints'][5])
    control = agent.run_step()

    timesteps = 0
    create_dataset()
    save_data(timesteps, obs, env.ego)

    while True:
        control = agent.run_step()
        obs, r, done, info = env.raw_step(control)

        timesteps += 1
        save_data(timesteps, obs, env.ego)

        agent.set_destination(info['waypoints'][5])

        if done:
            print('reset !', timesteps)
            close_dataset()

            obs = env.reset()
            agent = BasicAgent(env.ego, target_speed=params['desired_speed'] * 3.6)
            control = carla.VehicleControl(throttle=float(0), steer=float(0), brake=float(0))
            obs, r, done, info = env.raw_step(control)
            agent.set_destination(info['waypoints'][5])
            control = agent.run_step()

            timesteps = 0
            create_dataset()
            save_data(timesteps, obs, env.ego)


if __name__ == '__main__':
    main()
