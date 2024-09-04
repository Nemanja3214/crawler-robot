import argparse
import os
import random
import time
from distutils.util import strtobool

import gym
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from torch.distributions.normal import Normal
from tensorboardX import SummaryWriter

import rospy


# def parse_args():
#     # fmt: off
#     parser = argparse.ArgumentParser()
#     parser.add_argument("--exp-name", type=str, default=os.path.basename(__file__).rstrip(".py"),
#         help="the name of this experiment")
#     parser.add_argument("--gym-id", type=str, default="HalfCheetahBulletEnv-v0",
#         help="the id of the gym environment")
#     parser.add_argument("--learning-rate", type=float, default=3e-4,
#         help="the learning rate of the optimizer")
#     parser.add_argument("--seed", type=int, default=1,
#         help="seed of the experiment")
#     parser.add_argument("--total-timesteps", type=int, default=2000000,
#         help="total timesteps of the experiments")
#     parser.add_argument("--torch-deterministic", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="if toggled, `torch.backends.cudnn.deterministic=False`")
#     parser.add_argument("--cuda", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="if toggled, cuda will be enabled by default")
#     parser.add_argument("--track", type=lambda x: bool(strtobool(x)), default=False, nargs="?", const=True,
#         help="if toggled, this experiment will be tracked with Weights and Biases")
#     parser.add_argument("--wandb-project-name", type=str, default="ppo-implementation-details",
#         help="the wandb's project name")
#     parser.add_argument("--wandb-entity", type=str, default=None,
#         help="the entity (team) of wandb's project")
#     parser.add_argument("--capture-video", type=lambda x: bool(strtobool(x)), default=False, nargs="?", const=True,
#         help="weather to capture videos of the agent performances (check out `videos` folder)")

#     # Algorithm specific arguments
#     parser.add_argument("--num-envs", type=int, default=1,
#         help="the number of parallel game environments")
#     parser.add_argument("--num-steps", type=int, default=2048,
#         help="the number of steps to run in each environment per policy rollout")
#     parser.add_argument("--anneal-lr", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="Toggle learning rate annealing for policy and value networks")
#     parser.add_argument("--gae", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="Use GAE for advantage computation")
#     parser.add_argument("--gamma", type=float, default=0.99,
#         help="the discount factor gamma")
#     parser.add_argument("--gae-lambda", type=float, default=0.95,
#         help="the lambda for the general advantage estimation")
#     parser.add_argument("--num-minibatches", type=int, default=32,
#         help="the number of mini-batches")
#     parser.add_argument("--update-epochs", type=int, default=10,
#         help="the K epochs to update the policy")
#     parser.add_argument("--norm-adv", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="Toggles advantages normalization")
#     parser.add_argument("--clip-coef", type=float, default=0.2,
#         help="the surrogate clipping coefficient")
#     parser.add_argument("--clip-vloss", type=lambda x: bool(strtobool(x)), default=True, nargs="?", const=True,
#         help="Toggles whether or not to use a clipped loss for the value function, as per the paper.")
#     parser.add_argument("--ent-coef", type=float, default=0.0,
#         help="coefficient of the entropy")
#     parser.add_argument("--vf-coef", type=float, default=0.5,
#         help="coefficient of the value function")
#     parser.add_argument("--max-grad-norm", type=float, default=0.5,
#         help="the maximum norm for the gradient clipping")
#     parser.add_argument("--target-kl", type=float, default=None,
#         help="the target KL divergence threshold")
#     args = parser.parse_args()
#     args.batch_size = int(args.num_envs * args.num_steps)
#     args.minibatch_size = int(args.batch_size // args.num_minibatches)
#     # fmt: on
#     return args


def make_env(gym_id, seed):
    def thunk():
        env = gym.make(gym_id)
        env = gym.wrappers.RecordEpisodeStatistics(env)
        env = gym.wrappers.ClipAction(env)
        env = gym.wrappers.NormalizeObservation(env)
        env = gym.wrappers.TransformObservation(env, lambda obs: np.clip(obs, -10, 10))
        env = gym.wrappers.NormalizeReward(env)
        env = gym.wrappers.TransformReward(env, lambda reward: np.clip(reward, -10, 10))
        env.seed(seed)
        env.action_space.seed(seed)
        env.observation_space.seed(seed)
        return env

    return thunk


def layer_init(layer, std=np.sqrt(2), bias_const=0.0):
    torch.nn.init.orthogonal_(layer.weight, std)
    torch.nn.init.constant_(layer.bias, bias_const)
    return layer


class Agent(nn.Module):
    def __init__(self, envs):
        super(Agent, self).__init__()
        self.critic = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 1), std=1.0),
        )
        self.actor_mean = nn.Sequential(
            layer_init(nn.Linear(np.array(envs.single_observation_space.shape).prod(), 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, 64)),
            nn.Tanh(),
            layer_init(nn.Linear(64, np.prod(envs.single_action_space.shape)), std=0.01),
        )
        self.actor_logstd = nn.Parameter(torch.zeros(1, np.prod(envs.single_action_space.shape)))

    def get_value(self, x):
        return self.critic(x)

    def get_action_and_value(self, x, action=None):
        action_mean = self.actor_mean(x)
        action_logstd = self.actor_logstd.expand_as(action_mean)
        action_std = torch.exp(action_logstd)
        probs = Normal(action_mean, action_std)
        if action is None:
            action = probs.sample()
        return action, probs.log_prob(action).sum(1), probs.entropy().sum(1), self.critic(x)

def save():
    global agent
    dir = rospy.get_param("result_dir")
    torch.save(agent, dir + '/Hexapod-v0.pth')

import hexapod_env
global agent
def run(log_dir):
    global agent
    rospy.on_shutdown(save)
    gym_id = 'Hexapod-v0'
    exp_name = "def params"
    seed = int(time.time())
    run_name = f"{gym_id}__{exp_name}__{seed}__{int(time.time())}"
    writer = SummaryWriter(f"{log_dir}{run_name}")
   
    anneal_lr = True
    gae = True
    target_kl = None
    ent_coef = 0.0
    lr = 3e-4
    total_timesteps = 250000
    num_steps = 2048
    gamma = 0.99
    num_minibatches = 32
    gae_lambda = 0.95
    update_epochs = 10
    clip_coef = 0.2
    norm_adv = True
    clip_vloss = True
    vf_coef = 0.5
    max_grad_norm = 0.5
    batch_size = int(1 * num_steps)



    minibatch_size = int(batch_size // num_minibatches)
    parms = {
            "lr": lr,
            "num_steps": num_steps,
            "total_timesteps": total_timesteps,
            "batch_size":  batch_size,
            "anneal_lr":  anneal_lr,
            "gae":  gae,
            "gamma":  gamma,
            "gae_lambda":  gae_lambda,
            "minibatch_size":  minibatch_size,
            "update_epochs":  update_epochs,
            "clip_coef":  clip_coef,
            "norm_adv":  norm_adv,
            "clip_vloss":  clip_vloss,
            "ent_coef":  ent_coef,
            "vf_coef":  vf_coef,
            "max_grad_norm":  max_grad_norm,
            "target_kl":  target_kl
    }

    writer.add_text(
        "hyperparameters",
        "|param|value|\n|-|-|\n%s" % ("\n".join([f"|{key}|{value}|" for key, value in parms.items()])),
    )

    # TRY NOT TO MODIFY: seeding
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    torch.backends.cudnn.deterministic = True

    device = torch.device("cpu")

    # env setup
    envs = gym.vector.SyncVectorEnv(
        [make_env(gym_id, seed)]
    )
    assert isinstance(envs.single_action_space, gym.spaces.Box), "only continuous action space is supported"
    agent = Agent(envs).to(device)

    optimizer = optim.Adam(agent.parameters(), lr=lr, eps=1e-5)


    # ALGO Logic: Storage setup
    obs = torch.zeros((num_steps, 1) + envs.single_observation_space.shape).to(device)
    actions = torch.zeros((num_steps, 1) + envs.single_action_space.shape).to(device)
    logprobs = torch.zeros((num_steps, 1)).to(device)
    rewards = torch.zeros((num_steps, 1)).to(device)
    dones = torch.zeros((num_steps, 1)).to(device)
    values = torch.zeros((num_steps, 1)).to(device)

    # TRY NOT TO MODIFY: start the game
    global_step = 0
    start_time = time.time()
    next_obs = torch.Tensor(envs.reset()).to(device)
    next_done = torch.zeros(1).to(device)
    num_updates = total_timesteps // batch_size

    for update in range(1, num_updates + 1):
        # Annealing the rate if instructed to do so.
        if anneal_lr:
            frac = 1.0 - (update - 1.0) / num_updates
            lrnow = frac * lr
            optimizer.param_groups[0]["lr"] = lrnow

        for step in range(0, num_steps):
            global_step += 1 * 1
            obs[step] = next_obs
            dones[step] = next_done

            # ALGO LOGIC: action logic
            with torch.no_grad():
                action, logprob, _, value = agent.get_action_and_value(next_obs)
                values[step] = value.flatten()
            actions[step] = action
            logprobs[step] = logprob

            # TRY NOT TO MODIFY: execute the game and log data.
            next_obs, reward, done, info = envs.step(action.cpu().numpy())
            rewards[step] = torch.tensor(reward).to(device).view(-1)
            next_obs, next_done = torch.Tensor(next_obs).to(device), torch.Tensor(done).to(device)

            for item in info:
                if "episode" in item.keys():
                    print(f"global_step={global_step}, episodic_return={item['episode']['r']}")
                    writer.add_scalar("charts/episodic_return", item["episode"]["r"], global_step)
                    writer.add_scalar("charts/episodic_length", item["episode"]["l"], global_step)
                    break

        # bootstrap value if not done
        with torch.no_grad():
            next_value = agent.get_value(next_obs).reshape(1, -1)
            if gae:
                advantages = torch.zeros_like(rewards).to(device)
                lastgaelam = 0
                for t in reversed(range(num_steps)):
                    if t == num_steps - 1:
                        nextnonterminal = 1.0 - next_done
                        nextvalues = next_value
                    else:
                        nextnonterminal = 1.0 - dones[t + 1]
                        nextvalues = values[t + 1]
                    delta = rewards[t] + gamma * nextvalues * nextnonterminal - values[t]
                    advantages[t] = lastgaelam = delta + gamma * gae_lambda * nextnonterminal * lastgaelam
                returns = advantages + values
            else:
                returns = torch.zeros_like(rewards).to(device)
                for t in reversed(range(num_steps)):
                    if t == num_steps - 1:
                        nextnonterminal = 1.0 - next_done
                        next_return = next_value
                    else:
                        nextnonterminal = 1.0 - dones[t + 1]
                        next_return = returns[t + 1]
                    returns[t] = rewards[t] + gamma * nextnonterminal * next_return
                advantages = returns - values

        # flatten the batch
        b_obs = obs.reshape((-1,) + envs.single_observation_space.shape)
        b_logprobs = logprobs.reshape(-1)
        b_actions = actions.reshape((-1,) + envs.single_action_space.shape)
        b_advantages = advantages.reshape(-1)
        b_returns = returns.reshape(-1)
        b_values = values.reshape(-1)

        # Optimizing the policy and value network
        b_inds = np.arange(batch_size)
        clipfracs = []
        for epoch in range(update_epochs):
            np.random.shuffle(b_inds)
            for start in range(0, batch_size, minibatch_size):
                end = start + minibatch_size
                mb_inds = b_inds[start:end]

                _, newlogprob, entropy, newvalue = agent.get_action_and_value(b_obs[mb_inds], b_actions[mb_inds])
                logratio = newlogprob - b_logprobs[mb_inds]
                ratio = logratio.exp()

                with torch.no_grad():
                    # calculate approx_kl http://joschu.net/blog/kl-approx.html
                    old_approx_kl = (-logratio).mean()
                    approx_kl = ((ratio - 1) - logratio).mean()
                    clipfracs += [((ratio - 1.0).abs() > clip_coef).float().mean().item()]

                mb_advantages = b_advantages[mb_inds]
                if norm_adv:
                    mb_advantages = (mb_advantages - mb_advantages.mean()) / (mb_advantages.std() + 1e-8)

                # Policy loss
                pg_loss1 = -mb_advantages * ratio
                pg_loss2 = -mb_advantages * torch.clamp(ratio, 1 - clip_coef, 1 + clip_coef)
                pg_loss = torch.max(pg_loss1, pg_loss2).mean()

                # Value loss
                newvalue = newvalue.view(-1)
                if clip_vloss:
                    v_loss_unclipped = (newvalue - b_returns[mb_inds]) ** 2
                    v_clipped = b_values[mb_inds] + torch.clamp(
                        newvalue - b_values[mb_inds],
                        -clip_coef,
                        clip_coef,
                    )
                    v_loss_clipped = (v_clipped - b_returns[mb_inds]) ** 2
                    v_loss_max = torch.max(v_loss_unclipped, v_loss_clipped)
                    v_loss = 0.5 * v_loss_max.mean()
                else:
                    v_loss = 0.5 * ((newvalue - b_returns[mb_inds]) ** 2).mean()

                entropy_loss = entropy.mean()
                loss = pg_loss - ent_coef * entropy_loss + v_loss * vf_coef

                optimizer.zero_grad()
                loss.backward()
                nn.utils.clip_grad_norm_(agent.parameters(), max_grad_norm)
                optimizer.step()

            if target_kl is not None:
                if approx_kl > target_kl:
                    break

        y_pred, y_true = b_values.cpu().numpy(), b_returns.cpu().numpy()
        var_y = np.var(y_true)
        explained_var = np.nan if var_y == 0 else 1 - np.var(y_true - y_pred) / var_y

        # TRY NOT TO MODIFY: record rewards for plotting purposes
        writer.add_scalar("charts/learning_rate", optimizer.param_groups[0]["lr"], global_step)
        writer.add_scalar("losses/value_loss", v_loss.item(), global_step)
        writer.add_scalar("losses/policy_loss", pg_loss.item(), global_step)
        writer.add_scalar("losses/entropy", entropy_loss.item(), global_step)
        writer.add_scalar("losses/old_approx_kl", old_approx_kl.item(), global_step)
        writer.add_scalar("losses/approx_kl", approx_kl.item(), global_step)
        writer.add_scalar("losses/clipfrac", np.mean(clipfracs), global_step)
        writer.add_scalar("losses/explained_variance", explained_var, global_step)
        print("SPS:", int(global_step / (time.time() - start_time)))
        writer.add_scalar("charts/SPS", int(global_step / (time.time() - start_time)), global_step)

    envs.close()
    writer.close()