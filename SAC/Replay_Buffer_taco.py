import os
from collections import deque
import random
import numpy as np
from torch.utils.data import Dataset, DataLoader
import torch


def shift_point_cloud(batch_data, shift_range=0.1):
    """ Randomly shift point cloud. Shift is per point cloud.
        Input: shift_range 0-0.15
          BxNx3 array, original batch of point clouds
        Return:
          BxNx3 array, shifted batch of point clouds
    """
    B, N, C = batch_data.shape
    shifts = np.random.uniform(-shift_range, shift_range, (B,3))    # 对每个batch的点云设置一个随机的移动偏差
    for batch_index in range(B):
        batch_data[batch_index,:,:] += shifts[batch_index,:]    # 每个点都进行移动
    return batch_data

class ReplayBuffer(Dataset):
    """Buffer to store environment transitions."""

    def __init__(self, goal_shape, obs_shape, pcd_shape,
                 action_shape, capacity, batch_size, device, image_size=84, transform=True,
                 ):
        self.capacity = capacity
        self.batch_size = batch_size
        self.device = device
        self.transform = transform
        # the proprioceptive obs is stored as float32, pixels obs as uint8
        obs_dtype = np.float32 if len(obs_shape) == 1 else np.uint8
        self.goal = np.empty((capacity, goal_shape), dtype=np.float32)  # (100000, 24)
        self.next_goal = np.empty((capacity, goal_shape), dtype=np.float32)  # (100000, 24)
        self.obses = np.empty((capacity, obs_shape[0], obs_shape[1], obs_shape[2]), dtype=obs_dtype)
        self.next_obses = np.empty((capacity, obs_shape[0], obs_shape[1], obs_shape[2]), dtype=obs_dtype)
        self.pcd = np.empty((capacity, pcd_shape[0], pcd_shape[1]), dtype=np.float16)
        self.next_pcd = np.empty((capacity, pcd_shape[0], pcd_shape[1]), dtype=np.float16)
        self.actions = np.empty((capacity, action_shape), dtype=np.float32)
        self.rewards = np.empty((capacity, 1), dtype=np.float32)
        self.not_dones = np.empty((capacity, 1), dtype=np.float32)

        self.nstep = 3
        self.multistep = 3
        self.idx = 0
        self.last_save = 0
        self.full = False

    def add(self, goal, pcd, action, reward, next_goal, next_pcd, done):

        np.copyto(self.goal[self.idx], goal)
        if pcd.shape != (5200, 3):
            if pcd.shape[0] > 5200:
                pcd = pcd[:5200, :]  # 取前 5200 行，保留所有列
        np.copyto(self.next_goal[self.idx], next_goal)
        np.copyto(self.pcd[self.idx], pcd)
        np.copyto(self.actions[self.idx], action)
        np.copyto(self.rewards[self.idx], reward)
        if next_pcd.shape != (5200, 3):
            if next_pcd.shape[0] > 5200:
                next_pcd = next_pcd[:5200, :]  # 取前 5200 行，保留所有列
        np.copyto(self.next_pcd[self.idx], next_pcd)
        np.copyto(self.not_dones[self.idx], not done)

        self.idx = (self.idx + 1) % self.capacity
        self.full = self.full or self.idx == 0

    def sample_cpc(self):
        idxs = np.random.randint(
            0, self.capacity - self.multistep if self.full else self.idx - self.multistep, size=self.batch_size
        )

        goal = self.goal[idxs]
        next_goal = self.next_goal[idxs]
        pcd = self.pcd[idxs]
        next_pcd = self.next_pcd[idxs]
        pcd_anchor = shift_point_cloud(pcd)
        pcd_pos = shift_point_cloud(pcd)
        next_pcd = shift_point_cloud(next_pcd)
        r_next_pcd = self.pcd[idxs + self.multistep]
        r_next_pcd = shift_point_cloud(r_next_pcd)

        goal = torch.as_tensor(goal, device=self.device).float()
        next_goal = torch.as_tensor(next_goal, device=self.device).float()
        pcd = torch.as_tensor(pcd_anchor, device=self.device).float()
        next_pcd = torch.as_tensor(next_pcd, device=self.device).float()
        pcd_pos = torch.as_tensor(pcd_pos, device=self.device).float()
        r_next_pcd = torch.as_tensor(r_next_pcd, device=self.device).float()

        action_seq = np.concatenate([self.actions[idxs + i][None, :] for i in range(self.multistep)]).transpose(1, 0, 2)
        action_seq = torch.as_tensor(action_seq, device=self.device).float()
        actions = torch.as_tensor(self.actions[idxs], device=self.device)
        rewards = torch.as_tensor(self.rewards[idxs], device=self.device)
        not_dones = torch.as_tensor(self.not_dones[idxs], device=self.device)

        pcd_list = []
        next_pcd_list = []
        r_next_pcd_lsit = []
        pcd_pos_list = []
        for sample in pcd:
            pcd_list.append(sample)
        for sample in next_pcd:
            next_pcd_list.append(sample)
        for sample in r_next_pcd:
            r_next_pcd_lsit.append(sample)
        for sample in pcd_pos:
            pcd_pos_list.append(sample)
        return goal, pcd_list, actions, rewards, next_goal, next_pcd_list, not_dones, pcd_pos_list, action_seq, r_next_pcd_lsit



    def save(self, save_dir):
        if self.idx == self.last_save:
            return
        path = os.path.join(save_dir, '%d_%d.pt' % (self.last_save, self.idx))
        payload = [
            self.obses[self.last_save:self.idx],
            self.next_obses[self.last_save:self.idx],
            self.actions[self.last_save:self.idx],
            self.rewards[self.last_save:self.idx],
            self.not_dones[self.last_save:self.idx]
        ]
        self.last_save = self.idx
        torch.save(payload, path)

    def load(self, save_dir):
        chunks = os.listdir(save_dir)
        chucks = sorted(chunks, key=lambda x: int(x.split('_')[0]))
        for chunk in chucks:
            start, end = [int(x) for x in chunk.split('.')[0].split('_')]
            path = os.path.join(save_dir, chunk)
            payload = torch.load(path)
            assert self.idx == start
            self.obses[start:end] = payload[0]
            self.next_obses[start:end] = payload[1]
            self.actions[start:end] = payload[2]
            self.rewards[start:end] = payload[3]
            self.not_dones[start:end] = payload[4]
            self.idx = end

    def __getitem__(self, idx):
        idx = np.random.randint(
            0, self.capacity if self.full else self.idx, size=1
        )
        idx = idx[0]
        obs = self.obses[idx]
        action = self.actions[idx]
        reward = self.rewards[idx]
        next_obs = self.next_obses[idx]
        not_done = self.not_dones[idx]

        if self.transform:
            obs = self.transform(obs)
            next_obs = self.transform(next_obs)

        return obs, action, reward, next_obs, not_done

    def __len__(self):
        return self.capacity
