import itertools

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
from torch.nn import LayerNorm
import utils
from utils import PositionalEmbedding
from point_utils.pointnet import PointNet
from voxelnet import VoxelNet
LOG_FREQ = 10000

def gaussian_logprob(noise, log_std):
    """Compute Gaussian log probability."""
    residual = (-0.5 * noise.pow(2) - log_std).sum(-1, keepdim=True)
    return residual - 0.5 * np.log(2 * np.pi) * noise.size(-1)

def squash(mu, pi, log_pi):
    """Apply squashing function.
    See appendix C from https://arxiv.org/pdf/1812.05905.pdf.
    """
    mu = torch.tanh(mu)
    if pi is not None:
        pi = torch.tanh(pi)
    if log_pi is not None:
        log_pi -= torch.log(F.relu(1 - pi.pow(2)) + 1e-6).sum(-1, keepdim=True)
    return mu, pi, log_pi

def soft_update_params(net, target_net, tau):
    for param, target_param in zip(net.parameters(), target_net.parameters()):
        target_param.data.copy_(
            tau * param.data + (1 - tau) * target_param.data
        )

def weight_init(m):
    """Custom weight init for Conv2D and Linear layers."""
    if isinstance(m, nn.Linear):
        nn.init.orthogonal_(m.weight.data)
        m.bias.data.fill_(0.0)
    elif isinstance(m, nn.Conv2d) or isinstance(m, nn.ConvTranspose2d):
        # delta-orthogonal init from https://arxiv.org/pdf/1806.05393.pdf
        assert m.weight.size(2) == m.weight.size(3)
        m.weight.data.fill_(0.0)
        m.bias.data.fill_(0.0)
        mid = m.weight.size(2) // 2
        gain = nn.init.calculate_gain('relu')
        nn.init.orthogonal_(m.weight.data[:, :, mid, mid], gain)

class Actor(nn.Module):
    def __init__(self, obs_shape, action_shape, hidden_dim, encoder_type,
                 encoder_feature_dim, goal_dim, log_std_min, log_std_max, num_layers, num_filters):
        super().__init__()

        # self.pointnet = PointNet()
        # self.voxelNet = voxelNet
        self.conv = nn.Sequential(
            nn.AdaptiveMaxPool2d(1),
            nn.Flatten(),
            nn.Linear(128, 50),
            nn.Tanh()
        )
        self.log_std_min = log_std_min
        self.log_std_max = log_std_max
        self.trunk = nn.Sequential(
            nn.Linear(int(encoder_feature_dim * 2), hidden_dim), nn.ReLU(),
            nn.Linear(hidden_dim, hidden_dim), nn.ReLU(),
            nn.Linear(hidden_dim, 2 * action_shape)
        )
        self.embedding = nn.Linear(goal_dim, encoder_feature_dim)
        self.outputs = dict()
        # self.apply(weight_init)

    def forward(
            self, goal, obs, compute_pi=True, compute_log_pi=True, detach_encoder=False
    ):
        # [1, 512, 3]
        obs = self.conv(obs)
        goal = self.embedding(goal)                   # [1, 50]
        obs = torch.cat([obs, goal], 1)          # [B, 150]
        mu, log_std = self.trunk(obs).chunk(2, dim=-1)  # [B, 4]

        # constrain log_std inside [log_std_min, log_std_max]
        log_std = torch.tanh(log_std)
        log_std = self.log_std_min + 0.5 * (
                self.log_std_max - self.log_std_min
        ) * (log_std + 1)

        self.outputs['mu'] = mu
        self.outputs['std'] = log_std.exp()

        if compute_pi:
            std = log_std.exp()
            noise = torch.randn_like(mu)
            pi = mu + noise * std
        else:
            pi = None
            entropy = None

        if compute_log_pi:
            log_pi = gaussian_logprob(noise, log_std)
        else:
            log_pi = None

        mu, pi, log_pi = squash(mu, pi, log_pi)

        return mu, pi, log_pi, log_std

    def log(self, writer, step, log_freq=LOG_FREQ):
        if step % log_freq != 0:
            return

        writer.add_scalar('train_actor/q1', self.outputs['mu'], step)
        writer.add_scalar('train_std/std', self.outputs['std'], step)

class QFunction(nn.Module):
    def __init__(self, goal_dim, obs_dim, action_dim, hidden_dim):
        super().__init__()
        self.trunk = nn.Sequential(
            nn.Linear(obs_dim + action_dim + goal_dim, hidden_dim), nn.ReLU(True),
            nn.Linear(hidden_dim, hidden_dim), nn.ReLU(True),
            nn.Linear(hidden_dim, 1)
        )
    def forward(self, goal, obs, action):
        assert obs.size(0) == action.size(0)
        obs_action = torch.cat([obs, action, goal], dim=1)
        return self.trunk(obs_action)

class Critic(nn.Module):
    def __init__(
            self,  goal_dim, obs_shape, action_shape, hidden_dim, encoder_type,
            encoder_feature_dim, num_layers, num_filters
    ):
        super().__init__()
        # self.pointnet = PointNet()
        # self.voxelNet = voxelNet
        self.conv = nn.Sequential(
            nn.AdaptiveMaxPool2d(1),
            nn.Flatten(),
            nn.Linear(128, 50),
            nn.Tanh()
        )
        self.Q1 = QFunction(
            goal_dim, encoder_feature_dim, 3, hidden_dim,
        )
        self.Q2 = QFunction(
            goal_dim, encoder_feature_dim, 3, hidden_dim,
        )
        self.outputs = dict()
        # self.apply(weight_init)

    def forward(self, goal, obs, action, act_tok, detach_encoder=False):
        # detach_encoder allows to stop gradient propogation to encoder
        # obs = self.pointnet(obs)
        obs = self.conv(obs)
        if act_tok is not None:
            action = act_tok(action)
        q1 = self.Q1(goal, obs, action)
        q2 = self.Q2(goal, obs, action)

        self.outputs['q1'] = q1
        self.outputs['q2'] = q2

        return q1, q2

    def log(self, writer, step, log_freq= LOG_FREQ):
        if step % log_freq != 0:
            return

        writer.add_scalar('train_critic/q1', self.outputs['q1'], step)
        writer.add_scalar('train_critic/q2', self.outputs['q2'], step)

class TACO(nn.Module):
    def __init__(self, feature_dim, latent_a_dim, hidden_dim, act_tok, encoder, multistep, device):
        super(TACO, self).__init__()

        self.multistep = multistep
        self.encoder = encoder
        self.device = device

        self.proj_sa = nn.Sequential(
            nn.Linear(feature_dim + latent_a_dim*multistep, hidden_dim),
            nn.ReLU(True),
            nn.Linear(hidden_dim, feature_dim)
        )

        self.conv = nn.Sequential(
            nn.AdaptiveMaxPool2d(1),
            nn.Flatten(),
            nn.Linear(128, 50),
            nn.Tanh()
        )
        self.act_tok = act_tok

        self.global_classifier = nn.Sequential(
            nn.Linear(feature_dim, hidden_dim), nn.ReLU(True),
            nn.Linear(hidden_dim, feature_dim))


        self.global_final_classifier = nn.Sequential(
            nn.Linear(feature_dim, hidden_dim), nn.ReLU(True),
            nn.Linear(hidden_dim, feature_dim))


        # self.reward = nn.Sequential(
        #     nn.Linear(feature_dim + latent_a_dim*multistep, hidden_dim),
        #     nn.ReLU(True),
        #     nn.Linear(hidden_dim, 1)
        # )

        self.W = nn.Parameter(torch.rand(feature_dim, feature_dim))
        # self.apply(weight_init())

    def encode(self, x, ema=False):
        """
        Encoder: z_t = e(x_t)
        :param x: x_t, x y coordinates
        :return: z_t, value in r2
        """
        if ema:
            with torch.no_grad():
                z_out = self.conv(self.encoder(x))
        else:
            z_out = self.conv(self.encoder(x))
        return z_out

    def project_sa(self, s, a):
        a = self.act_tok(a, seq=True)
        x = torch.concat([s, a], dim=-1)
        return self.proj_sa(x)

    def compute_logits(self, z_a, z_pos):
        """
        - compute (B,B) matrix z_a (W z_pos.T)
        - positives are all diagonal elements
        - negatives are all other elements
        - to compute loss use multiclass cross entropy with identity matrix for labels
        """
        Wz = torch.matmul(self.W, z_pos.T)
        logits = torch.matmul(z_a, Wz)
        logits = logits - torch.max(logits, 1)[0][:, None]
        return logits

    def spr_loss(self, latents, target_latents, no_grad=False):
        if no_grad:
            with torch.no_grad():
                global_latnet = self.global_classifier(latents)
                global_latnet = self.global_final_classifier(global_latnet)
        else:
            global_latnet = self.global_classifier(latents)
            global_latnet = self.global_final_classifier(global_latnet)

        with torch.no_grad():
            global_targets = self.global_classifier(target_latents)

        loss = self.norm_cos_loss(global_latnet, global_targets, mean=False).mean()
        return loss


    def norm_cos_loss(self, f_x1s, f_x2s, mean=True):

        f_x1 = F.normalize(f_x1s.float(), p=2., dim=-1, eps=1e-3)
        f_x2 = F.normalize(f_x2s.float(), p=2., dim=-1, eps=1e-3)
        loss = 2 - 2 * (f_x1 * f_x2).sum(dim=-1)
        loss = torch.mean(loss) if mean else loss
        return loss


class ActionEncoding(nn.Module):
    def __init__(self, action_dim, latent_action_dim, multistep):
        super().__init__()
        self.action_dim = action_dim
        self.action_tokenizer = nn.Sequential(
            nn.Linear(action_dim, 64), nn.Tanh(),
            nn.Linear(64, latent_action_dim)
        )
        self.action_seq_tokenizer = nn.Sequential(
            nn.Linear(latent_action_dim*multistep, latent_action_dim*multistep),
            nn.LayerNorm(latent_action_dim*multistep), nn.Tanh()
        )
        self.apply(weight_init)

    def forward(self, action, seq=False):
        if seq:
            batch_size = action.shape[0]
            action = self.action_tokenizer(action)
            action = action.reshape(batch_size, -1)
            return self.action_seq_tokenizer(action)
        else:
            return self.action_tokenizer(action)

class BarlowLoss(nn.Module):
    def __init__(self, lmbda=0.005, reduction='mean'):
        super(BarlowLoss, self).__init__()
        self.lmbda = lmbda
        self.reduction = reduction

    def _off_diagonal(self, x):
        n, m = x.shape
        assert n == m
        return x.flatten()[:-1].view(n - 1, n + 1)[:, 1:].flatten()

    def forward(self, z1, z2):
        n, d = z1.shape
        z1 = (z1 - z1.mean(0)) / z1.std(0)
        z2 = (z2 - z2.mean(0)) / z2.std(0)

        cor = torch.mm(z1.T, z2)
        cor.div_(n)

        on_diag = torch.diagonal(cor).add_(-1).pow_(2).sum()
        off_diag = self._off_diagonal(cor).pow_(2).sum()

        loss = on_diag + self.lmbda * off_diag

        if self.reduction == 'mean':
            return loss
        else:
            raise ValueError

class CtmrSacAgent(object):
    def __init__(self,
                 obs_shape,
                 device,
                 action_shape,
                 writer,
                 hidden_dim=256,
                 goal_dim=4,
                 discount=0.99,
                 init_temperature=0.01,
                 alpha_lr=1e-3,
                 alpha_beta=0.9,
                 actor_lr=1e-3,
                 actor_beta=0.9,
                 actor_log_std_min=-10,
                 actor_log_std_max=2,
                 actor_update_freq=2,
                 critic_lr=1e-3,
                 critic_beta=0.9,
                 critic_tau=0.005,
                 critic_target_update_freq=2,
                 encoder_type='pixel',
                 encoder_feature_dim=50,
                 encoder_lr=1e-3,
                 encoder_tau=0.005,
                 num_layers=4,
                 num_filters=32,
                 cpc_update_freq=1,
                 log_interval=100,
                 detach_encoder=False,
                 curl_latent_dim=128,
                 num_attn_layer=2,
                 actor_attach_encoder=False,
                 actor_coeff=1.,
                 ):
        self.device = device
        self.discount = discount
        self.critic_tau = critic_tau
        self.encoder_tau = encoder_tau
        self.actor_update_freq = actor_update_freq
        self.critic_target_update_freq = critic_target_update_freq
        self.cpc_update_freq = cpc_update_freq
        self.log_interval = log_interval
        self.image_size = obs_shape[-1]
        self.curl_latent_dim = curl_latent_dim
        self.detach_encoder = detach_encoder
        self.encoder_type = encoder_type
        self.actor_attach_encoder = actor_attach_encoder
        self.actor_coeff = actor_coeff
        self.latent_a_dim = 3
        self.multistep = 3
        self.writer = writer
        self.curl = True
        self.reward = True
        self.voxelNet = VoxelNet()
        self.actor = Actor(obs_shape, action_shape, hidden_dim, encoder_type,
            encoder_feature_dim, goal_dim, actor_log_std_min, actor_log_std_max,
            num_layers, num_filters).to(self.device)
        self.critic = Critic(
            goal_dim, obs_shape, action_shape, hidden_dim, encoder_type,
            encoder_feature_dim, num_layers, num_filters
        ).to(self.device)
        self.critic_target = Critic(
            goal_dim, obs_shape, action_shape, hidden_dim, encoder_type,
            encoder_feature_dim, num_layers, num_filters
        ).to(self.device)
        self.act_tok = ActionEncoding(2, self.latent_a_dim, self.multistep)
        self.TACO = TACO(encoder_feature_dim, self.latent_a_dim, hidden_dim, self.act_tok, self.voxelNet, self.multistep,
                         device).to(device)
        self.critic_target.load_state_dict(self.critic.state_dict())
        self.log_alpha = torch.tensor(np.log(init_temperature)).to(self.device)
        self.log_alpha.requires_grad = True
        self.target_entropy = -np.prod(action_shape)

        parameters = itertools.chain(self.voxelNet.parameters(),
                                     self.act_tok.parameters())

        self.actor_optimizer = torch.optim.Adam(
            self.actor.parameters(), lr=actor_lr, betas=(actor_beta, 0.999)
        )
        self.critic_optimizer = torch.optim.Adam(
            self.critic.parameters(), lr=critic_lr, betas=(critic_beta, 0.999)
        )
        self.log_alpha_optimizer = torch.optim.Adam(
            [self.log_alpha], lr=alpha_lr, betas=(alpha_beta, 0.999)
        )
        self.taco_optimizer = torch.optim.Adam(self.TACO.parameters(),
                                               lr=encoder_lr)
        self.encoder_optimizer = torch.optim.Adam(parameters, lr=encoder_lr)
        self.cross_entropy_loss = nn.CrossEntropyLoss()
        self.train()
        self.critic_target.train()

    def train(self, training=True):
        self.training = training
        self.voxelNet.train(training)
        self.actor.train(training)
        self.critic.train(training)
        self.TACO.train(training)

    @property
    def alpha(self):
        return self.log_alpha.exp()

    def select_action(self, goal, obs):
        pcd = []
        obs = torch.as_tensor(obs, dtype=torch.float32).to(self.device)
        # obs = obs.unsqueeze(0)
        pcd.append(obs)
        pcd = self.voxelNet(pcd)
        goal = torch.FloatTensor(goal).to(self.device)
        goal = goal.unsqueeze(0)
        with torch.no_grad():
            mu, _, _, _ = self.actor(goal,
                pcd, compute_pi=False, compute_log_pi=False
            )
            return mu.cpu().data.numpy().flatten()

    def sample_action(self, goal, obs):
        # obs [1024, 3]
        pcd = []
        obs = torch.as_tensor(obs, dtype=torch.float32).to(self.device)
        # obs = obs.unsqueeze(0)
        pcd.append(obs)
        pcd = self.voxelNet(pcd)
        goal = torch.FloatTensor(goal).to(self.device)
        goal = goal.unsqueeze(0)
        with torch.no_grad():
            mu, pi, _, _ = self.actor(goal, pcd, compute_log_pi=False)
            return pi.cpu().data.numpy().flatten()

    def update_critic(self, goal, pcd, action, reward, next_goal, next_pcd, not_done,  step):
        with torch.no_grad():
            _, policy_action, log_pi, _ = self.actor(next_goal, next_pcd)
            target_Q1, target_Q2 = self.critic_target(next_goal, next_pcd, policy_action, self.act_tok)
            target_V = torch.min(target_Q1,
                                 target_Q2) - self.alpha.detach() * log_pi
            target_Q = reward + (not_done * self.discount * target_V)

        # get current Q estimates
        current_Q1, current_Q2 = self.critic(goal,
            pcd, action, self.act_tok)
        critic_loss = F.mse_loss(current_Q1,
                                 target_Q) + F.mse_loss(current_Q2, target_Q)

        if step % self.log_interval == 0:
            self.writer.add_scalar('target_Q/loss', target_Q.mean().item(), step)
            self.writer.add_scalar('current_Q1/loss', current_Q1.mean().item(), step)
            self.writer.add_scalar('current_Q2/loss', current_Q2.mean().item(), step)
            self.writer.add_scalar('train_critic/loss', critic_loss.item(), step)
        # Optimize the critic
        self.encoder_optimizer.zero_grad(set_to_none=True)
        self.critic_optimizer.zero_grad(set_to_none=True)
        critic_loss.backward()
        self.critic_optimizer.step()
        self.encoder_optimizer.step()

    def update_actor_and_alpha(self, goal, pcd,  step):
        # detach encoder, so we don't update it with the actor loss
        _, pi, log_pi, log_std = self.actor(goal, pcd)
        actor_Q1, actor_Q2 = self.critic(goal, pcd, pi, self.act_tok)

        actor_Q = torch.min(actor_Q1, actor_Q2)
        actor_loss = (self.alpha.detach() * log_pi - actor_Q).mean()

        if step % self.log_interval == 0:
            self.writer.add_scalar('train_actor/loss', actor_loss, step)
            self.writer.add_scalar('train_actor/target_entropy', self.target_entropy, step)

        entropy = 0.5 * log_std.shape[1] * \
                  (1.0 + np.log(2 * np.pi)) + log_std.sum(dim=-1)

        # optimize the actor
        self.actor_optimizer.zero_grad(set_to_none=True)
        actor_loss.backward()
        self.actor_optimizer.step()

        self.log_alpha_optimizer.zero_grad()
        alpha_loss = (self.alpha *
                      (-log_pi - self.target_entropy).detach()).mean()
        alpha_loss.backward()
        self.log_alpha_optimizer.step()
        if step % self.log_interval == 0:
            self.writer.add_scalar('train_alpha/loss', alpha_loss.item(), step)
            self.writer.add_scalar('train_alpha/value', self.alpha, step)

    def update_taco(self, pcd_anchor, action, action_seq, reward, pcd_pos, r_next_pcd, step):
        z_a = self.TACO.encode(pcd_anchor)
        z_pos = self.TACO.encode(pcd_pos, ema=True)

        if self.curl:
            logits = self.TACO.compute_logits(z_a, z_pos)
            curl_loss = self.TACO.spr_loss(z_a, z_pos)
            # labels = torch.arange(logits.shape[0]).long().to(self.device)
            # curl_loss = self.cross_entropy_loss(logits, labels)
            # reg_loff_fn = BarlowLoss()
            # t1 = F.normalize(self.TACO.proj_s(z_a), dim=-1, p=2)
            # t2 = F.normalize(self.TACO.proj_s(z_pos), dim=-1, p=2)
            # barlow_loss = reg_loff_fn(t1, t2)
        else:
            barlow_loss = torch.tensor(0.)

        # action_seq_en = self.TACO.act_tok(action_seq, seq=True)

        # if self.reward:
        #     reward_pred = self.TACO.reward(torch.concat([z_a, action_seq_en], dim=-1))
        #     reward_loss = F.mse_loss(reward_pred, reward)
        # else:
        #     reward_loss = torch.Tensor(0.).float()

        next_z = self.TACO.encode(r_next_pcd, ema=True)
        curr_za = self.TACO.project_sa(z_a, action_seq)
        logits = self.TACO.compute_logits(curr_za, next_z)
        labels = torch.arange(logits.shape[0]).long().to(self.device)
        taco_loss = self.cross_entropy_loss(logits, labels)

        # loss = taco_loss
        self.taco_optimizer.zero_grad()
        # loss.backward()
        (taco_loss + curl_loss).backward()
        self.taco_optimizer.step()
        if step % self.log_interval == 0:
            self.writer.add_scalar('taco_loss/loss', taco_loss, step)
            self.writer.add_scalar('curl_loss/loss', curl_loss, step)
            # self.writer.add_scalar('reward_loss/loss', reward_loss, step)

    def update(self, replay_buffer, step):
        state, pcd, action, reward, next_state, next_pcd, not_done, pcd_pos, action_seq, r_next_pcd = replay_buffer.sample_cpc()

        pcd_en = self.voxelNet(pcd)
        with torch.no_grad():
            next_pcd_en = self.voxelNet(next_pcd)

        self.update_critic(state, pcd_en, action, reward, next_state, next_pcd_en, not_done, step)

        if step % self.log_interval == 0:
            self.writer.add_scalar('train/batch_reward', reward.mean(), step)

        if step % self.actor_update_freq == 0:
            self.update_actor_and_alpha(state, pcd_en.detach(), step)

        self.update_taco(pcd, action, action_seq, reward, pcd_pos, r_next_pcd, step)
        if step % self.critic_target_update_freq == 0:
            soft_update_params(
                self.critic, self.critic_target, self.critic_tau
            )


    def save(self, model_dir, step):
        torch.save(
            self.actor.state_dict(), '%s/actor_%s.pt' % (model_dir, step)
        )
        torch.save(
            self.critic.state_dict(), '%s/critic_%s.pt' % (model_dir, step)
        )
        torch.save(
            self.TACO.state_dict(), '%s/taco_%s.pt' % (model_dir, step)
        )
    def load(self, model_dir, step):
        self.actor.load_state_dict(
            torch.load('%s/actor_%s.pt' % (model_dir, step))
        )
        self.critic.load_state_dict(
            torch.load('%s/critic_%s.pt' % (model_dir, step))
        )
        self.TACO.load_state_dict(
            torch.load('%s/taco_%s.pt' % (model_dir, step))
        )

def make_agent(obs_shape, device,action_shape, writer, args):
    return CtmrSacAgent(
        obs_shape=obs_shape,
        device=device,
        action_shape=action_shape,
        writer=writer,
        hidden_dim=args.hidden_dim,
        goal_dim=args.goal_dim,
        discount=args.discount,
        init_temperature=args.init_temperature,
        alpha_lr=args.alpha_lr,
        alpha_beta=args.alpha_beta,
        actor_lr=args.actor_lr,
        actor_beta=args.actor_beta,
        actor_log_std_min=args.actor_log_std_min,
        actor_log_std_max=args.actor_log_std_max,
        actor_update_freq=args.actor_update_freq,
        critic_lr=args.critic_lr,
        critic_beta=args.critic_beta,
        critic_tau=args.critic_tau,
        critic_target_update_freq=args.critic_target_update_freq,
        encoder_type=args.encoder_type,
        encoder_feature_dim=args.encoder_feature_dim,
        encoder_lr=args.encoder_lr,
        encoder_tau=args.encoder_tau,
        num_layers=args.num_layers,
        num_filters=args.num_filters,
        log_interval=args.log_interval,
        detach_encoder=args.detach_encoder,
        curl_latent_dim=args.curl_latent_dim,
        num_attn_layer=args.num_attn_layer,
        actor_attach_encoder=args.actor_attach_encoder,
        actor_coeff=args.actor_coeff,
    )