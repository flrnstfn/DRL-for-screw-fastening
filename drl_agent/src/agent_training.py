#! /usr/bin/env python3

import gymnasium as gym
import rospy
import os
import training_environment
import torch as th
from std_msgs.msg import Float64
from stable_baselines3.common.env_checker import check_env
from stable_baselines3 import PPO
from stable_baselines3.common.policies import ActorCriticPolicy
from stable_baselines3.common.monitor import Monitor 
from stable_baselines3.common.callbacks import BaseCallback
from sb3_contrib import RecurrentPPO

class TensorboardCallBack(BaseCallback):

    def __init__ (self, verbose = 2):
        super().__init__(verbose)

    def _on_rollout_end(self) -> None:
        info = self.locals['infos'][0]  
        success_value = info.get('success')
        if success_value is not None:
            self.logger.record("successful_episodes", success_value)
        return True
    
    def _on_step(self) -> bool:
        return True
    
if __name__ == '__main__':

    rospy.init_node('screwfastening_training', log_level=rospy.DEBUG)

    #Create the environment
    env = gym.make("ScrewFastening-v1")
    rospy.logdebug ( "Gym environment done")

    #Load parameters from ros parameter server
    log_dir                     = rospy.get_param("/log_dir")
    save_dir                    = rospy.get_param("/save_dir")
    policy                      = rospy.get_param("/policy")
    learning_rate               = rospy.get_param("/learning_rate")
    n_steps                     = rospy.get_param("/n_steps")
    batch_size                  = rospy.get_param("/batch_size")
    n_epochs                    = rospy.get_param("/n_epochs")
    gamma                       = rospy.get_param("/gamma")
    gae_lambda                  = rospy.get_param("/gae_lambda")
    clip_range                  = rospy.get_param("/clip_range")
    #clip_range_vf              = rospy.get_param("/clip_range_vf")
    ent_coef                    = rospy.get_param("/ent_coef")
    vf_coef                     = rospy.get_param("/vf_coef")
    verbose                     = rospy.get_param("/verbose")
    total_timesteps             = rospy.get_param("/total_timesteps")
    tb_log_name                 = rospy.get_param("/tb_log_name")
    progress_bar                = rospy.get_param("/progress_bar")
    policy_net_size             = rospy.get_param("/policy_net_size")
    vf_net_size                 = rospy.get_param("/vf_net_size")
    activation_fnc              = rospy.get_param("/activation_fnc")

    if activation_fnc == "TanH":
        activation_fn=th.nn.Tanh
        print(1)
    elif activation_fnc == "ReLU":
        activation_fn=th.nn.ReLU
    
    policy_kwargs = dict(activation_fn=activation_fn, net_arch=dict(pi= policy_net_size, vf= vf_net_size))

    #Create path for logging
    current_file_dir = os.path.dirname(os.path.abspath(__file__))
    rel_path_log_dir = os.path.join(current_file_dir, '..', 'training_results')
    
    #Put environment into monitor wrapper
    env = Monitor(env, rel_path_log_dir)

    #Create neural networks for actor and critic
    #network = ActorCriticPolicy()

    #Create callback class
    callback = TensorboardCallBack()

    #Create the PPO Agent
    ppo_agent = PPO(policy, env, learning_rate, n_steps, batch_size, n_epochs, gamma, gae_lambda, clip_range, ent_coef=ent_coef, vf_coef= vf_coef, tensorboard_log=rel_path_log_dir, policy_kwargs=policy_kwargs, verbose=verbose)
    #ppo_agent = RecurrentPPO(policy, env, learning_rate, n_steps, batch_size, n_epochs, gamma, gae_lambda, clip_range, ent_coef=ent_coef, vf_coef= vf_coef, tensorboard_log=rel_path_log_dir, policy_kwargs=policy_kwargs, verbose=verbose)
    #ppo_agent = PPO.load("/home/florian/Documents/Trained_Agents/agent_12.zip")
    #ppo_agent.set_env(env)
    rospy.logdebug ( "Agent initialized")
    ppo_agent.learn(total_timesteps, tb_log_name=tb_log_name, progress_bar=progress_bar, callback=callback)
    ppo_agent.save(save_dir)
