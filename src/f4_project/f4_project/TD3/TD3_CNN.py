import copy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F


device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# Paper: https://arxiv.org/abs/1802.09477


class Actor(nn.Module):
    def __init__(self, input_image_shape, goal_dim, action_dim, max_action):
        """
        Initializes the Actor network with CNN layers for image processing
        and concatenates goal coordinates with visual features.

        Args:
            input_image_shape (tuple): Shape of the input image (channels, height, width).
                                       Example: (1, 84, 84) for a grayscale 84x84 image.
            goal_dim (int): Dimensionality of the goal coordinates (e.g., 2 for (x, y)).
            action_dim (int): Number of possible actions.
            max_action (float): Maximum absolute value for actions (for continuous action spaces).
        """
        super(Actor, self).__init__()

        # --- CNN Layers for Image Processing ---
        self.conv1 = nn.Conv2d(
            in_channels=input_image_shape[0],
            out_channels=32,
            kernel_size=3,
            stride=1,
            padding=1 
        )
        self.conv2 = nn.Conv2d(
            in_channels=32,
            out_channels=64,
            kernel_size=3,
            stride=1,
            padding=1 
        )

        # Calculate the flattened size of the CNN output dynamically
        dummy_input = torch.zeros(1, *input_image_shape)
        x = F.relu(self.conv1(dummy_input))
        x = F.relu(self.conv2(x))
        flattened_cnn_size = x.view(x.size(0), -1).size(1)

        # --- Fully Connected Layers ---
        # The first linear layer now takes the flattened CNN output + goal_dim
        self.l1 = nn.Linear(flattened_cnn_size + goal_dim, 800) # Adjusted input size here
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, action_dim)

        self.max_action = max_action
        
    def forward(self, state_image, goal_coords):
        """
        Forward pass through the Actor network, combining image features and goal coordinates.

        Args:
            state_image (torch.Tensor): The input observation image, 
                                        expected as (batch_size, channels, height, width).
            goal_coords (torch.Tensor): The goal coordinates, 
                                         expected as (batch_size, goal_dim).
        
        Returns:
            torch.Tensor: The predicted actions.
        """
        # Process image through CNN layers
        a = F.relu(self.conv1(state_image))
        a = F.relu(self.conv2(a))

        # Flatten the CNN output features
        a = a.view(a.size(0), -1) 

        # Concatenate flattened image features with goal coordinates
        # Ensure both are (batch_size, features) before concatenation
        combined_features = torch.cat([a, goal_coords], dim=1) 

        # Apply fully connected layers to the combined features
        a = F.relu(self.l1(combined_features)) # Pass combined features to the first linear layer
        a = F.relu(self.l2(a))
        a = torch.tanh(self.l3(a)) # tanh for continuous actions between -1 and 1

        return self.max_action * a


class Critic(nn.Module):
    def __init__(self, input_image_shape, goal_dim, action_dim):
        """
        Initializes the Critic network with CNN layers for image processing
        and concatenates goal coordinates and actions with visual features.

        Args:
            input_image_shape (tuple): Shape of the input image (channels, height, width).
                                       Example: (1, 84, 84) for a grayscale 84x84 image.
            goal_dim (int): Dimensionality of the goal coordinates (e.g., 2 for (x, y)).
            action_dim (int): Dimensionality of the action space.
        """
        super(Critic, self).__init__()

        # --- Shared CNN Layers for Image Processing (for both Q1 and Q2) ---
        self.conv1 = nn.Conv2d(
            in_channels=input_image_shape[0],
            out_channels=32,
            kernel_size=3,
            stride=1,
            padding=1 
        )
        self.conv2 = nn.Conv2d(
            in_channels=32,
            out_channels=64,
            kernel_size=3,
            stride=1,
            padding=1 
        )

        # Calculate the flattened size of the CNN output dynamically
        dummy_input = torch.zeros(1, *input_image_shape)
        x = F.relu(self.conv1(dummy_input))
        x = F.relu(self.conv2(x))
        flattened_cnn_size = x.view(x.size(0), -1).size(1)

        # The input dimension for the first linear layer of each Q-network
        # will be: (flattened_cnn_output_size + goal_dim + action_dim)
        combined_feature_dim = flattened_cnn_size + goal_dim + action_dim

        # --- Q1 Architecture ---
        self.l1 = nn.Linear(combined_feature_dim, 800)
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, 1) # Output a single Q-value

        # --- Q2 Architecture ---
        self.l4 = nn.Linear(combined_feature_dim, 800)
        self.l5 = nn.Linear(800, 600)
        self.l6 = nn.Linear(600, 1) # Output a single Q-value

    def forward(self, state_image, goal_coords, action):
        """
        Forward pass through the Critic network to estimate Q-values.

        Args:
            state_image (torch.Tensor): The input observation image,
                                        expected as (batch_size, channels, height, width).
            goal_coords (torch.Tensor): The goal coordinates,
                                         expected as (batch_size, goal_dim).
            action (torch.Tensor): The action taken,
                                    expected as (batch_size, action_dim).

        Returns:
            tuple: Two Q-values (q1, q2) estimated by the two Q-networks.
        """
        # Process image through shared CNN layers
        # Output: (Batch_size, 64, H_out2, W_out2)
        image_features = F.relu(self.conv1(state_image))
        image_features = F.relu(self.conv2(image_features))

        # Flatten the CNN output features
        image_features = image_features.view(image_features.size(0), -1)

        # Concatenate flattened image features with goal coordinates
        # Result: (batch_size, flattened_cnn_size + goal_dim)
	
        combined_state_features = torch.cat([image_features, goal_coords], dim=1)

        # Concatenate the combined state features with the action
        # This is the full input to the Q-networks: (batch_size, (state_features + action_dim))
        sa = torch.cat([combined_state_features, action], dim=1)

        # --- Q1 Forward Pass ---
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)

        # --- Q2 Forward Pass ---
        q2 = F.relu(self.l4(sa))
        q2 = F.relu(self.l5(q2))
        q2 = self.l6(q2)

        return q1, q2
	
    def Q1(self, state_image, goal_coords, action):
		 # Process image through shared CNN layers
        # Output: (Batch_size, 64, H_out2, W_out2)
        image_features = F.relu(self.conv1(state_image))
        image_features = F.relu(self.conv2(image_features))

        # Flatten the CNN output features
        image_features = image_features.view(image_features.size(0), -1)

        # Concatenate flattened image features with goal coordinates
        # Result: (batch_size, flattened_cnn_size + goal_dim)
	
        combined_state_features = torch.cat([image_features, goal_coords], dim=1)

        # Concatenate the combined state features with the action
        # This is the full input to the Q-networks: (batch_size, (state_features + action_dim))
        sa = torch.cat([combined_state_features, action], dim=1)

        # --- Q1 Forward Pass ---
        q1 = F.relu(self.l1(sa))
        q1 = F.relu(self.l2(q1))
        q1 = self.l3(q1)
		
        return q1


class TD3(object):
	def __init__(
		self,
		input_image_shape,
		goal_dim,
		action_dim,
		max_action,
		discount=0.99,
		tau=0.005,
		policy_noise=0.2,
		noise_clip=0.5,
		policy_freq=2
	):

		self.actor = Actor(input_image_shape, goal_dim, action_dim, max_action).to(device)
		self.actor_target = copy.deepcopy(self.actor)
		self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=3e-4)

		self.critic = Critic(input_image_shape, goal_dim, action_dim).to(device)
		self.critic_target = copy.deepcopy(self.critic)
		self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

		self.max_action = max_action
		self.discount = discount
		self.tau = tau
		self.policy_noise = policy_noise
		self.noise_clip = noise_clip
		self.policy_freq = policy_freq

		self.total_it = 0
	
	
	def select_action(self,state):
		state_goal = torch.FloatTensor(state["goal"].reshape(1, -1)).to(device)
		state_image = torch.FloatTensor(state["image"].reshape(1, 1, 32, -1)).to(device)
		action = self.actor(state_image,state_goal)
		return action.cpu().data.numpy().flatten()


	def train(self, replay_buffer, batch_size=256):
		self.total_it += 1

		# Sample replay buffer 
		state_image, state_goal, action, next_state_image, next_state_goal, reward, not_done = replay_buffer.sample(batch_size)

		with torch.no_grad():
			# Select action according to policy and add clipped noise
			noise = (
				torch.randn_like(action) * self.policy_noise
			).clamp(-self.noise_clip, self.noise_clip)
			
			next_action = (
				self.actor_target(next_state_image,next_state_goal) + noise
			).clamp(-self.max_action, self.max_action)

			# Compute the target Q value
			target_Q1, target_Q2 = self.critic_target(next_state_image,next_state_goal, next_action)
			target_Q = torch.min(target_Q1, target_Q2)
			Q_value = torch.mean(target_Q)
			target_Q = reward + not_done * self.discount * target_Q

		# Get current Q estimates
		current_Q1, current_Q2 = self.critic(state_image,state_goal, action)

		# Compute critic loss
		critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

		# Optimize the critic
		self.critic_optimizer.zero_grad()
		critic_loss.backward()   
		self.critic_optimizer.step()

		# Delayed policy updates
		if self.total_it % self.policy_freq == 0:

			# Compute actor losse
			actor_loss = -self.critic.Q1(state_image,state_goal, self.actor(state_image,state_goal)).mean()
			
			# Optimize the actor 
			self.actor_optimizer.zero_grad()
			actor_loss.backward()
			self.actor_optimizer.step()

			# Update the frozen target models
			for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
				target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

			for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
				target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
		return Q_value, critic_loss


	def save(self, filename):
		torch.save(self.critic.state_dict(), filename + "_critic")
		torch.save(self.critic_optimizer.state_dict(), filename + "_critic_optimizer")
		torch.save(self.actor.state_dict(), filename + "_actor")
		torch.save(self.actor_optimizer.state_dict(), filename + "_actor_optimizer")


	def load(self, filename):
		self.critic.load_state_dict(torch.load(filename + "_critic",weights_only=True))
		self.critic_optimizer.load_state_dict(torch.load(filename + "_critic_optimizer",weights_only=True))
		self.critic_target = copy.deepcopy(self.critic)

		self.actor.load_state_dict(torch.load(filename + "_actor",weights_only=True))
		self.actor_optimizer.load_state_dict(torch.load(filename + "_actor_optimizer",weights_only=True))
		self.actor_target = copy.deepcopy(self.actor)
		