#! /usr/bin/env python3

import gymnasium as gym
import rospy
import os
import training_environment
import matplotlib.pyplot as plt
import pickle
import csv
from std_msgs.msg import Float64
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.monitor import Monitor 
from stable_baselines3.common.callbacks import BaseCallback

def successful_eps_plot(list, filepath_plot):
    plt.plot(range(1, len(list)+1), list, marker='o')
    plt.xlabel('Number of Episodes')
    plt.ylabel('Number of successful episodes')
    plt.grid(True)
    plt.savefig(filepath_plot) #add the name of the file
    plt.close()

def reward_eps_plot(list, filepath_plot):
    plt.plot(range(1, len(list)+1), list, marker='o')
    plt.xlabel('Number of Episodes')
    plt.ylabel('Reward per Episode')
    plt.grid(True)
    plt.savefig(filepath_plot) #add the name of the file
    plt.close()

def max_force_eps_plot(list, filepath_plot):
    plt.plot(range(1, len(list)+1), list, marker='o')
    plt.xlabel('Number of Episodes')
    plt.ylabel('Max. TCP-Force in Episode')
    plt.grid(True)
    plt.savefig(filepath_plot) #add the name of the file
    plt.close()

def save_list(list, filepath_list):
    with open(filepath_list, 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Iteration', 'Value'])  # Header row
        for i, value in enumerate(list, start=1):
            writer.writerow([i, value])



if __name__ == '__main__':

    rospy.init_node('agent_evaluation node', log_level=rospy.DEBUG)

    #Create the environment
    env = gym.make("ScrewFastening-v1")
    rospy.logdebug ( "Gym environment done")
 
    #Create path for saving files
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    filepath_plot_suc = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_success_eps_plot.svg')
    filepath_list_suc = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_success_eps_list.csv')
    filepath_plot_rew = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_eward_plot.svg')
    filepath_list_rew = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_reward_list.csv')
    filepath_plot_maxf = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_max_force_plot.svg')
    filepath_list_maxf = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_max_force_list.csv')
    filepath_list_posx = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_posx_list.csv')
    filepath_list_posy = os.path.join(current_file_dir, '..', 'evaluation_results', 'a12_0002_actscl_0004_posy_list.csv')

    #Create the PPO Agent
    ppo_agent = PPO.load("/home/florian/Documents/Trained_Agents/agent_12.zip")
    #print(ppo_agent.get_parameters())
    ppo_agent.set_env(env)
    rospy.logdebug ("Agent initialized")

    #Init list
    list_success_eps = []
    list_reward_eps = []
    list_max_contact_force_eps =[]
    list_pos_x = []
    list_pos_y = []

    for i in range(100):
        rospy.logdebug(f"Epsiode: {i}")
        obs, info_reset = env.reset()
        total_reward = 0
        max_force = 0
        list_pos_x.append(info_reset['pos_x'])
        list_pos_y.append(info_reset['pos_y'])
        for n in range(200):
            action = ppo_agent.predict(observation=obs)
            print(action[0])
            obs, reward, terminated, truncated, info = env.step(action=action[0])
            print(obs)
            total_reward += reward
            if info['contact_force'] > max_force:
                max_force = info['contact_force']
            if terminated == True: 
                break
        print(info)
        list_success_eps.append(info['success'])
        list_reward_eps.append(total_reward)
        list_max_contact_force_eps.append(max_force)


    successful_eps_plot(list_success_eps, filepath_plot_suc)
    save_list(list_success_eps, filepath_list_suc)
    reward_eps_plot(list_reward_eps, filepath_plot_rew)
    save_list(list_reward_eps, filepath_list_rew)
    max_force_eps_plot(list_max_contact_force_eps, filepath_plot_maxf)
    save_list(list_max_contact_force_eps, filepath_list_maxf)    
    save_list(list_pos_x, filepath_list_posx)
    save_list(list_pos_y, filepath_list_posy)
