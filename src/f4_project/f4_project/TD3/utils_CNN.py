import numpy as np
import torch


class ReplayBuffer(object):
	def __init__(self, input_image_shape, goal_dim, action_dim, max_size=int(1e6)):
		self.max_size = max_size
		self.ptr = 0
		self.size = 0

		# Image data: (max_size, channels, height, width)
		self.state_image = np.zeros((max_size, *input_image_shape), dtype=np.float32)
		self.next_state_image = np.zeros((max_size, *input_image_shape), dtype=np.float32)

		# Goal data: (max_size, goal_dim)
		self.state_goal = np.zeros((max_size, goal_dim), dtype=np.float32)
		self.next_state_goal = np.zeros((max_size, goal_dim), dtype=np.float32)

		self.action = np.zeros((max_size, action_dim))
		self.reward = np.zeros((max_size, 1))
		self.not_done = np.zeros((max_size, 1))


		self.device = torch.device("cuda" if torch.cuda.is_available() else "cpu")


	def add(self, state_image, state_goal, action, next_state_image, next_state_goal, reward, done):
		self.state_image[self.ptr] = state_image
		self.state_goal[self.ptr] = state_goal
		self.action[self.ptr] = action
		self.next_state_image[self.ptr] = next_state_image
		self.next_state_goal[self.ptr] = next_state_goal
		self.reward[self.ptr] = reward
		self.not_done[self.ptr] = 1. - done

		# Detach the hidden state so that BPTT only goes through 1 timestep
	
		self.ptr = (self.ptr + 1) % self.max_size
		self.size = min(self.size + 1, self.max_size)


	def sample(self, batch_size):
		ind = np.random.randint(0, self.size, size=batch_size)

		s_i = torch.FloatTensor(
            self.state_image[ind]).to(self.device)
		s_g = torch.FloatTensor(
            self.state_goal[ind]).to(self.device)
		a = torch.FloatTensor(
			self.action[ind]).to(self.device)
		ns_i = torch.FloatTensor(
			self.next_state_image[ind]).to(self.device)
		ns_g = torch.FloatTensor(
			self.next_state_goal[ind]).to(self.device)

		r = torch.FloatTensor(
			self.reward[ind]).to(self.device)
		d = torch.FloatTensor(
			self.not_done[ind]).to(self.device)

		return (
			s_i,s_g,a,ns_i,ns_g,r,d
		)
	
	def save(self,filename):
		np.save(filename + "_state",self.state_image)
		np.save(filename + "_action",self.action)
		np.save(filename + "_next_state",self.next_state_image)
		np.save(filename + "_reward",self.reward)
		np.save(filename + "_not_done",self.not_done)

	
	def load(self,filename):
			self.state_image = np.load(filename + "_state.npy")
			self.action = np.load(filename + "_action.npy")
			self.next_state_image = np.load(filename + "_next_state.npy")
			self.reward = np.load(filename + "_reward.npy")
			self.not_done = np.load(filename + "_not_done.npy")
			self.ptr = np.max(np.nonzero(self.state_laser))
			print(self.ptr)
			