import os
import time
import numpy as np
import torch
import torch.nn as nn
from velodyne_env import GazeboEnv

from utils import IOStream
import argparse
from Ctmr_sac import make_agent
from Replay_Buffer import ReplayBuffer
from torch.utils.tensorboard import SummaryWriter
from Data_Augmentation import center_crop_image
# from logger import Logger
import utils
# /home/jjh/DRL-robot-navigation/TD3
dirPath = os.path.dirname(os.path.realpath(__file__))

def action_unnormalized(action, high, low):
    action = low + (action + 1.0) * 0.5 * (high - low)
    action = np.clip(action, low, high)
    return action

class eval_mode(object):
    def __init__(self, *models):
        self.models = models

    def __enter__(self):
        self.prev_states = []
        for model in self.models:
            self.prev_states.append(model.training)
            model.train(False)

    def __exit__(self, *args):
        for model, state in zip(self.models, self.prev_states):
            model.train(state)
        return False

def evaluate(eval_episodes=0, encoder_type='Pixel', image_size=84,sample_stochastically=False):
    all_ep_rewards = []
    step_count = []
    collison = 0
    success = 0
    for i in range(eval_episodes):
        state, obs = env.reset()
        done = False
        episode_reward = 0
        step = 0
        while not done and step < 501:
            if encoder_type == 'pixel':
                obs = obs[:, 20:120, 20:120]
                obs = center_crop_image(obs, image_size)
            with eval_mode(agent):
                if sample_stochastically:
                    action = agent.sample_action(state, obs)
                    unnorm_action = np.array([
                        action_unnormalized(action[0], ACTION_V_MAX, 0),
                        action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)
                    ])
                else:
                    action = agent.select_action(state, obs)
                    unnorm_action = np.array([
                        action_unnormalized(action[0], ACTION_V_MAX, 0),
                        action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)
                    ])
                state, obs, reward, done, target = env.step(unnorm_action)
                episode_reward += reward
                step += 1
            if reward < -90:
                collison += 1
            if target:
                success += 1
        all_ep_rewards.append(episode_reward)
        if target:
            step_count.append(step)
    mean_ep_reward = np.mean(all_ep_rewards)
    mean_step = np.mean(step_count)
    best_ep_reward = np.max(all_ep_rewards)
    avg_col = collison / eval_episodes
    avg_success = success / eval_episodes
    io.cprint("..............................................")
    io.cprint("Average Reward over %i Evaluation Episodes, Epoch %i: %f, %f, %f, %f" % (
    eval_episodes, epoch, mean_ep_reward, avg_col, avg_success, mean_step))
    io.cprint('Evaluation  Mean :%f Best : %f' % (mean_ep_reward, best_ep_reward))
    io.cprint("..............................................")
    return mean_ep_reward, best_ep_reward

if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    # environment
    parser.add_argument("--env_name", default="HalfCheetahBulletEnv-v0")
    parser.add_argument("--seed", default=0, type=int)
    parser.add_argument("--eval_freq", default=2e4, type=int, help="After how many steps to perform the evaluation 5e3")
    parser.add_argument("--max_ep", default=350, type=int)
    parser.add_argument("--max_timesteps", default=5e6, type=int, help="Maximum number of steps to perform")
    parser.add_argument("--random_near_obstacle", default=True, type=bool,
                        help="To take random actions near obstacles or not")
    parser.add_argument('--image_size', default=84, type=int)
    # replay buffer
    parser.add_argument('--replay_buffer_capacity', default=100000, type=int)
    # train
    parser.add_argument('--hidden_dim', default=1024, type=int)
    parser.add_argument('--goal_dim', default=4, type=int)
    parser.add_argument('--batch_size', default=64, type=int)
    parser.add_argument('--init_steps', default=1000, type=int)
    parser.add_argument('--latent_dim', default=100, type=int)
    parser.add_argument('--momentum_tau', default=0.05, type=float)

    # encoder
    parser.add_argument('--encoder_type', default='pixel', type=str)
    parser.add_argument('--encoder_feature_dim', default=50, type=int)
    parser.add_argument('--encoder_lr', default=1e-3, type=float)
    parser.add_argument('--encoder_tau', default=0.05, type=float)
    parser.add_argument('--num_layers', default=4, type=int)
    parser.add_argument('--num_attn_layer', default=1, type=int)
    parser.add_argument('--num_filters', default=32, type=int)
    parser.add_argument('--curl_latent_dim', default=128, type=int)
    # actor
    parser.add_argument('--actor_lr', default=1e-3, type=float)
    parser.add_argument('--actor_beta', default=0.9, type=float)
    parser.add_argument('--actor_log_std_min', default=-10, type=float)
    parser.add_argument('--actor_log_std_max', default=2, type=float)
    parser.add_argument('--actor_update_freq', default=2, type=int)
    # critic
    parser.add_argument('--critic_lr', default=1e-3, type=float)
    parser.add_argument('--critic_beta', default=0.9, type=float)
    parser.add_argument('--critic_tau', default=0.01, type=float)  # try 0.05 or 0.1
    parser.add_argument('--critic_target_update_freq', default=2, type=int)  # try to change it to 1 and retain 0.01 above
    # sac
    parser.add_argument('--discount', default=0.99, type=float)
    parser.add_argument('--init_temperature', default=0.1, type=float)
    parser.add_argument('--alpha_lr', default=1e-4, type=float)
    parser.add_argument('--alpha_beta', default=0.5, type=float)
    # misc
    parser.add_argument('--work_dir', default='.', type=str)
    parser.add_argument('--exp_suffix', default='', type=str)
    parser.add_argument('--actor_attach_encoder', default=False, action='store_true')
    parser.add_argument('--detach_encoder', default=False, action='store_true')
    parser.add_argument('--log_interval', default=100, type=int)
    parser.add_argument('--actor_coeff', default=1., type=float)
    parser.add_argument('--adam_warmup_step', type=float)
    parser.add_argument('--mtm_length', default=64, type=int)
    parser.add_argument('--encoder_annealling', default=False, action='store_true')
    parser.add_argument('--mtm_bsz', default=16, type=int)
    parser.add_argument('--mtm_ratio', default=0.5, type=float)
    parser.add_argument('--mtm_not_ema', default=False, action='store_true')
    parser.add_argument('--normalize_before', default=False, action='store_true')
    parser.add_argument('--dropout', default=0., type=float)
    parser.add_argument('--attention_dropout', default=0., type=float)
    parser.add_argument('--relu_dropout', default=0., type=float)
    args = parser.parse_args()

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")  # cuda or cpu
    # Create the network storage folders
    if not os.path.exists("./results"):
        os.makedirs("./results")

    io = IOStream('./results/output' + '/output_mask_view.log')
    env = GazeboEnv('multi_robot_scenario.launch', 1, 1, 1)
    time.sleep(5)
    # env.seed(seed)
    torch.manual_seed(args.seed)
    np.random.seed(args.seed)

    action_shape = 2
    obs_shape = (3, 84, 84)
    pre_aug_obs_shape = (3, 140, 140)
    replay_buffer_size = 100000
    laser_shape = 24
    episode, episode_reward = 0, 0
    save_model_replay = False

    # make directory
    # ts = time.gmtime()
    # ts = time.strftime("%m-%d", ts)
    # exp_suffix = '-' + args.exp_suffix if args.exp_suffix else ''
    # env_name = args.agent + '-' + args.domain_name + '-' + args.task_name
    # exp_name = env_name + '-' + ts + '-im' + str(args.image_size) + '-b' \
    #            + str(args.batch_size) + '-' + args.encoder_type \
    #            + exp_suffix + '-s' + str(args.seed)
    # args.work_dir = args.work_dir + '/' + exp_name
    #
    # utils.make_dir(args.work_dir)

    # Create a replay buffer
    replay_buffer = ReplayBuffer(
        laser_shape=laser_shape,
        obs_shape=pre_aug_obs_shape,
        action_shape=action_shape,
        capacity=args.replay_buffer_capacity,
        batch_size=args.batch_size,
        device=device,
        image_size=args.image_size,
        mtm_bsz=args.mtm_bsz,
        mtm_length=args.mtm_length,
        mtm_ratio=args.mtm_ratio
    )

    # replay_buffer = ReplayBuffer(
    #     laser_shape=laser_shape,
    #     obs_shape=pre_aug_obs_shape,
    #     action_shape=action_shape,
    #     capacity=replay_buffer_size,
    #     batch_size=128,
    #     device=device,
    #     image_size=84,
    #)

    ACTION_V_MIN = 0    # m/s
    ACTION_W_MIN = -1  # rad/s
    ACTION_V_MAX = 1  # m/s
    ACTION_W_MAX = 1  # rad/s
    # Create the network
    agent = make_agent(
        obs_shape=obs_shape,
        action_shape=action_shape,
        device=device,
        args=args
    )

    # L = Logger(args.work_dir, use_tb=args.save_tb)
    # agent.load(dirPath, 27)
    # Create evaluation data store
    evaluations = []
    timestep = 0
    timesteps_since_eval = 0
    episode_num = 0
    done = True
    epoch = 1
    frame_step = 3
    count_rand_actions = 0
    random_action = []
    writer = SummaryWriter(dirPath + '/evaluations/', flush_secs=1, max_queue=1)
    # Begin the training loop

    while timestep < args.max_timesteps:  # 5000000
        # On termination of episode
        # mean, best = evaluate(3, 'pixel')
        if timesteps_since_eval >= args.eval_freq:
            timesteps_since_eval %= args.eval_freq
            mean, best = evaluate(100, 'ViTEncoder')
            writer.add_scalar('Reward mean', mean, epoch)
            writer.add_scalar('Reward best', best, epoch)
            if save_model_replay:
                if timestep % (1000) == 0:
                    agent.save(dirPath, epoch)
                    agent.save_curl(dirPath, epoch)
                    replay_buffer.save(dirPath + '/replay_memory/')
                    print('saved model and replay memory', timestep)
            done = True
            save_model_replay = True
            epoch += 1

        if done:
            print("*********************************")
            print('Episode: ' + str(episode_num) + ' training')
            print('Reward average per ep: ' + str(episode_reward))

            state, obs = env.reset()  # (24) (3, 84, 84)
            done = False
            episode_reward = 0
            episode_timesteps = 0
            episode_num += 1

        if timestep < args.init_steps:
            action = np.array([
                np.random.uniform(-1, 1),
                np.random.uniform(-1, 1)
                ])
            unnorm_action = np.array([
                action_unnormalized(action[0], ACTION_V_MAX, 0),
                action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)
                ])
        else:
            with eval_mode(agent):
                action = agent.sample_action(state, obs)
                unnorm_action = np.array([
                    action_unnormalized(action[0], ACTION_V_MAX, ACTION_V_MIN),
                    action_unnormalized(action[1], ACTION_W_MAX, ACTION_W_MIN)
                ])
        unnorm_action=[0,0]
        # If the robot is facing an obstacle, randomly force it to take a consistent random action.
        # This is done to increase exploration in situations near obstacles.
        # Training can also be performed without it
        # 如果机器人面临障碍物，则随机强制其采取一致的随机动作。
        # 这样做是为了在障碍物附近的情况下增加探索。
        # 也可以在没有它的情况下进行培训
        if args.random_near_obstacle:
            if np.random.uniform(0, 1) > 0.85 and min(state[4:-8]) < 0.6 and count_rand_actions < 1:
                count_rand_actions = np.random.randint(8, 15)
                random_action = np.random.uniform(-1, 1, 2)

            if count_rand_actions > 0:
                count_rand_actions -= 1
                action = random_action
                action[0] = -1

        if timestep >= args.init_steps:
            agent.update(replay_buffer, timestep)

        # Update action to fall in range [0,1] for linear velocity and [-1,1] for angular velocity
        next_state, next_obs, reward, done, target = env.step(unnorm_action)

        done_bool = 0 if episode_timesteps + 1 == args.max_ep else int(done)  # 500
        done = 1 if episode_timesteps + 1 == args.max_ep else int(done)
        episode_reward += reward

        # Save the tuple in replay buffer
        replay_buffer.add(state, obs, action, reward, next_state, next_obs, done)
        #  (24,) (3, 100, 100) (2, ) ()  (24,) (3, 100, 100)
        # Update the counters
        state = next_state
        obs = next_obs
        episode_timesteps += 1
        timestep += 1
        timesteps_since_eval += 1

