import copy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# Implementation of Twin Delayed Deep Deterministic Policy Gradients (TD3)
# This version is modified to use a Variational Autoencoder (VAE) based encoder.

# --- Variational Autoencoder (VAE) Components ---

class VAE_Encoder(nn.Module):
    def __init__(self, input_image_shape, latent_dim):
        """
        Initializes the VAE Encoder.
        This network flattens the image and compresses it into a
        latent distribution (mean and log-variance).

        Args:
            input_image_shape (tuple): Shape of the input image (channels, height, width).
            latent_dim (int): Dimensionality of the output latent space.
        """
        super(VAE_Encoder, self).__init__()
        
        self.input_dim = int(np.prod(input_image_shape))
        self.latent_dim = latent_dim
        
        # Shared MLP layers
        self.net = nn.Sequential(
            nn.Linear(self.input_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
        )
        
        # Output heads for mean (mu) and log-variance (log_var)
        self.fc_mu = nn.Linear(512, self.latent_dim)
        self.fc_log_var = nn.Linear(512, self.latent_dim)

    def reparameterize(self, mu, log_var):
        """
        Reparameterization trick: z = mu + std * epsilon
        """
        std = torch.exp(0.5 * log_var)
        epsilon = torch.randn_like(std)
        return mu + epsilon * std

    def forward(self, state_image):
        """
        Forward pass through the Encoder.

        Args:
            state_image (torch.Tensor): Input image (batch_size, channels, height, width).
        
        Returns:
            tuple: (z, mu, log_var)
                z (torch.Tensor): The sampled latent vector (batch_size, latent_dim).
                mu (torch.Tensor): The latent mean (batch_size, latent_dim).
                log_var (torch.Tensor): The latent log variance (batch_size, latent_dim).
        """
        # Flatten the image: (batch_size, C, H, W) -> (batch_size, C*H*W)
        flattened_image = state_image.view(state_image.size(0), -1)
        
        # Pass through the shared MLP
        h = self.net(flattened_image)
        
        # Get mu and log_var
        mu = self.fc_mu(h)
        log_var = self.fc_log_var(h)
        
        # Get latent vector z using reparameterization trick
        z = self.reparameterize(mu, log_var)
        
        return z, mu, log_var


class Decoder(nn.Module):
    def __init__(self, latent_dim, output_image_shape):
        """
        Initializes the MLP Decoder.
        This network reconstructs the image from the latent vector.
        """
        super(Decoder, self).__init__()
        
        self.output_shape = output_image_shape
        self.output_dim = int(np.prod(output_image_shape))
        self.latent_dim = latent_dim
        
        self.net = nn.Sequential(
            nn.Linear(self.latent_dim, 512),
            nn.ReLU(),
            nn.Linear(512, 512),
            nn.ReLU(),
            nn.Linear(512, self.output_dim),
            nn.Sigmoid()  # Use Sigmoid if images are normalized [0, 1]
        )

    def forward(self, latent_vector):
        """
        Forward pass through the Decoder.

        Args:
            latent_vector (torch.Tensor): Latent vector (batch_size, latent_dim).
        
        Returns:
            torch.Tensor: Reconstructed image (batch_size, channels, height, width).
        """
        # Pass through the MLP
        flattened_image = self.net(latent_vector)
        
        # Reshape to image dimensions
        reconstructed_image = flattened_image.view(flattened_image.size(0), *self.output_shape)
        return reconstructed_image


class VAE(nn.Module):
    def __init__(self, image_shape, latent_dim):
        """
        Combines the VAE_Encoder and Decoder.
        Useful for pre-training the Encoder.
        """
        super(VAE, self).__init__()
        self.encoder = VAE_Encoder(image_shape, latent_dim)
        self.decoder = Decoder(latent_dim, image_shape)
        self.image_shape_flat = int(np.prod(image_shape))

    def forward(self, state_image):
        z, mu, log_var = self.encoder(state_image)
        reconstructed_image = self.decoder(z)
        return reconstructed_image, mu, log_var

    def loss_function(self, reconstruction, original_image, mu, log_var):
        """
        Calculates the VAE loss (Reconstruction + KL Divergence).
        """
        # Reconstruction loss (using Binary Cross Entropy for [0,1] images)
        # Use F.mse_loss if images are not normalized to [0,1]
        BCE = F.binary_cross_entropy(
            reconstruction.view(-1, self.image_shape_flat), 
            original_image.view(-1, self.image_shape_flat), 
            reduction='sum'
        )
        
        # KL Divergence loss
        # KLD = -0.5 * sum(1 + log(sigma^2) - mu^2 - sigma^2)
        KLD = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp())
        
        return BCE + KLD


# --- Modified Actor and Critic to use VAE Encoder ---

class Actor(nn.Module):
    def __init__(self, input_image_shape, goal_dim, action_dim, max_action, latent_dim):
        """
        Initializes the Actor network with a VAE Encoder.
        """
        super(Actor, self).__init__()

        # --- VAE Encoder for Image Processing ---
        # We only need the encoder part for the RL agent
        self.encoder = VAE_Encoder(input_image_shape, latent_dim)

        # --- MLP for Goal Coordinates ---
        self.goal_mlp = nn.Sequential(
            nn.Linear(goal_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU()
        )
        goal_mlp_output_size = 64 

        # --- Fully Connected Layers for Fusion ---
        self.l1 = nn.Linear(latent_dim + goal_mlp_output_size, 800)
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, action_dim)

        self.max_action = max_action
        
    def forward(self, state_image, goal_coords):
        """
        Forward pass through the Actor network.
        
        Note: We get z, mu, and log_var from the encoder, but only
        use the sampled latent vector 'z' for action selection.
        The mu and log_var are ignored here (only used for VAE loss).
        """
        # Process image through the VAE Encoder
        image_features_z, _, _ = self.encoder(state_image)

        # Process goal coordinates through the goal MLP
        embedded_goal_features = self.goal_mlp(goal_coords)
        
        # Concatenate latent image features (z) with embedded goal features
        combined_features = torch.cat([image_features_z, embedded_goal_features], dim=1) 

        # Apply fully connected layers to the combined features
        a = F.relu(self.l1(combined_features))
        a = F.relu(self.l2(a))
        a = torch.tanh(self.l3(a))
        
        return self.max_action * a


class Critic(nn.Module):
    def __init__(self, input_image_shape, goal_dim, action_dim, latent_dim):
        """
        Initializes the Critic network with a VAE Encoder.
        """
        super(Critic, self).__init__()

        # --- Shared VAE Encoder for Image Processing ---
        self.encoder = VAE_Encoder(input_image_shape, latent_dim)
        
        # --- MLP for Goal Coordinates ---
        self.goal_mlp = nn.Sequential(
            nn.Linear(goal_dim, 32),
            nn.ReLU(),
            nn.Linear(32, 64),
            nn.ReLU()
        )
        goal_mlp_output_size = 64 

        # Combined feature dimension
        combined_feature_dim = latent_dim + goal_mlp_output_size + action_dim

        # --- Q1 Architecture ---
        self.l1 = nn.Linear(combined_feature_dim, 800)
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, 1)

        # --- Q2 Architecture ---
        self.l4 = nn.Linear(combined_feature_dim, 800)
        self.l5 = nn.Linear(800, 600)
        self.l6 = nn.Linear(600, 1)

    def forward(self, state_image, goal_coords, action):
        """
        Forward pass through the Critic network.
        Only uses the sampled 'z' vector from the encoder.
        """
        # Process image through shared VAE Encoder
        image_features_z, _, _ = self.encoder(state_image)

        # Process goal coordinates through the goal MLP
        embedded_goal_features = self.goal_mlp(goal_coords)

        # Concatenate latent image features (z), embedded goal, and action
        sa = torch.cat([image_features_z, embedded_goal_features, action], dim=1)

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
        """
        Forward pass through the Q1 network only.
        """
        # Process image through shared VAE Encoder
        image_features_z, _, _ = self.encoder(state_image)

        # Process goal coordinates through the goal MLP
        embedded_goal_features = self.goal_mlp(goal_coords)

        # Concatenate latent image features (z), embedded goal, and action
        sa = torch.cat([image_features_z, embedded_goal_features, action], dim=1)

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
        latent_dim=256,  # Latent dim for the VAE
        encoder_weights_path="vae_encoder.pth", # <-- New argument
        discount=0.99,
        tau=0.005,
        policy_noise=0.2,
        noise_clip=0.5,
        policy_freq=2
    ):

        # --- Pass latent_dim to VAE-based Actor and Critic ---
        self.actor = Actor(input_image_shape, goal_dim, action_dim, max_action, latent_dim).to(device)
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=3e-4)

        self.critic = Critic(input_image_shape, goal_dim, action_dim, latent_dim).to(device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=3e-4)

        # --- Load pre-trained encoder weights if provided ---
        if encoder_weights_path is not None:
            self.load_pretrained_encoder(encoder_weights_path)
            
        self.max_action = max_action
        self.discount = discount
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq

        self.total_it = 0
    
    
    def select_action(self, state):
        # This function remains unchanged.
        state_goal = torch.FloatTensor(state["goal"].reshape(1, -1)).to(device)
        state_image = torch.FloatTensor(state["image"].reshape(1, *state["image"].shape)).to(device)
        
        action = self.actor(state_image, state_goal)
        return action.cpu().data.numpy().flatten()


    def train(self, replay_buffer, batch_size=256):
        # This function remains unchanged.
        # It trains the actor and critic end-to-end.
        # The VAE_Encoder is trained *only* based on the RL losses.
        
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
            Q_value = torch.mean(target_Q) # For logging
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

            # Compute actor loss
            # Note: The actor loss trains the VAE_Encoder as well
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
        
        return Q_value.item(), critic_loss.item()


    def save(self, filename):
        # This function remains unchanged.
        torch.save(self.critic.state_dict(), filename + "_critic")
        torch.save(self.critic_optimizer.state_dict(), filename + "_critic_optimizer")
        torch.save(self.actor.state_dict(), filename + "_actor")
        torch.save(self.actor_optimizer.state_dict(), filename + "_actor_optimizer")


    def load(self, filename):
        # This function remains unchanged.
        self.critic.load_state_dict(torch.load(filename + "_critic"))
        self.critic_optimizer.load_state_dict(torch.load(filename + "_critic_optimizer"))
        self.critic_target = copy.deepcopy(self.critic)

        self.actor.load_state_dict(torch.load(filename + "_actor"))
        self.actor_optimizer.load_state_dict(torch.load(filename + "_actor_optimizer"))
        self.actor_target = copy.deepcopy(self.actor)

    def load_pretrained_encoder(self, filename):
        """
        Helper function to load weights from a pre-trained VAE encoder.
        """
        print(f"Loading pre-trained encoder weights from {filename}")
        # Load weights, ensuring they are mapped to the correct device
        encoder_weights = torch.load(filename, map_location=device)
        
        # Load weights into both actor's and critic's encoders
        self.actor.encoder.load_state_dict(encoder_weights)
        self.critic.encoder.load_state_dict(encoder_weights)
        # We also load them into the target networks
        self.actor_target.encoder.load_state_dict(encoder_weights)
        self.critic_target.encoder.load_state_dict(encoder_weights)