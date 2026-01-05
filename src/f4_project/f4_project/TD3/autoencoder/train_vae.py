#!/usr/bin/env python3

"""
This ROS2 node subscribes to an image topic, collects images into a buffer,
and trains a Variational Autoencoder (VAE) on batches of these images.

This is intended for "pre-training" the VAE_Encoder. The saved encoder weights
can then be loaded by the TD3 agent.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import torch.optim as optim

# --- VAE Model Definitions ---
# These classes are copied here to make this node self-contained.
# They should match the definitions in the TD3_VAE.py file.

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

class VAE_Encoder(nn.Module):
    def __init__(self, input_image_shape, latent_dim):
        super(VAE_Encoder, self).__init__()
        self.input_dim = int(np.prod(input_image_shape))
        self.latent_dim = latent_dim
        
        self.net = nn.Sequential(
            nn.Linear(self.input_dim, 512), nn.ReLU(),
            nn.Linear(512, 512), nn.ReLU(),
        )
        self.fc_mu = nn.Linear(512, self.latent_dim)
        self.fc_log_var = nn.Linear(512, self.latent_dim)

    def reparameterize(self, mu, log_var):
        std = torch.exp(0.5 * log_var)
        epsilon = torch.randn_like(std)
        return mu + epsilon * std

    def forward(self, state_image):
        flattened_image = state_image.view(state_image.size(0), -1)
        h = self.net(flattened_image)
        mu = self.fc_mu(h)
        log_var = self.fc_log_var(h)
        z = self.reparameterize(mu, log_var)
        return z, mu, log_var

class Decoder(nn.Module):
    def __init__(self, latent_dim, output_image_shape):
        super(Decoder, self).__init__()
        self.output_shape = output_image_shape
        self.output_dim = int(np.prod(output_image_shape))
        self.latent_dim = latent_dim
        
        self.net = nn.Sequential(
            nn.Linear(self.latent_dim, 512), nn.ReLU(),
            nn.Linear(512, 512), nn.ReLU(),
            nn.Linear(512, self.output_dim),
            nn.Sigmoid()  # Assumes images are normalized [0, 1]
        )

    def forward(self, latent_vector):
        flattened_image = self.net(latent_vector)
        reconstructed_image = flattened_image.view(flattened_image.size(0), *self.output_shape)
        return reconstructed_image

class VAE(nn.Module):
    def __init__(self, image_shape, latent_dim):
        super(VAE, self).__init__()
        self.encoder = VAE_Encoder(image_shape, latent_dim)
        self.decoder = Decoder(latent_dim, image_shape)
        self.image_shape_flat = int(np.prod(image_shape))

    def forward(self, state_image):
        z, mu, log_var = self.encoder(state_image)
        reconstructed_image = self.decoder(z)
        return reconstructed_image, mu, log_var

    def loss_function(self, reconstruction, original_image, mu, log_var):
        # Reconstruction loss
        BCE = F.binary_cross_entropy(
            reconstruction.view(-1, self.image_shape_flat), 
            original_image.view(-1, self.image_shape_flat), 
            reduction='sum'
        )
        
        # KL Divergence loss
        KLD = -0.5 * torch.sum(1 + log_var - mu.pow(2) - log_var.exp())
        
        return BCE + KLD

# --- ROS2 Node ---

class VAETrainerNode(Node):
    def __init__(self):
        super().__init__('vae_trainer_node')
        
        # Declare parameters with default values
        self.declare_parameter('image_topic', '/simple_drone/camera/depth/image_raw')
        self.declare_parameter('latent_dim', 32)
        self.declare_parameter('image_shape', [1, 32, 32]) # [C, H, W]
        self.declare_parameter('learning_rate', 1e-3)
        self.declare_parameter('batch_size', 128)
        self.declare_parameter('save_path', 'vae_encoder.pth')
        self.declare_parameter('save_every_n_batches', 100)
        
        # Get parameters
        image_topic = self.get_parameter('image_topic').value
        latent_dim = self.get_parameter('latent_dim').value
        self.image_shape = self.get_parameter('image_shape').value
        lr = self.get_parameter('learning_rate').value
        self.batch_size = self.get_parameter('batch_size').value
        self.save_path = self.get_parameter('save_path').value
        self.save_every_n = self.get_parameter('save_every_n_batches').value
        
        # Ensure image_shape is a list/tuple
        if isinstance(self.image_shape, str):
            import ast
            self.image_shape = ast.literal_eval(self.image_shape)
        
        self.target_height = self.image_shape[1]
        self.target_width = self.image_shape[2]
        self.channels = self.image_shape[0]
        
        # VAE Model
        self.vae = VAE(self.image_shape, latent_dim).to(device)
        self.optimizer = optim.Adam(self.vae.parameters(), lr=lr)
        
        # ROS Subscriber and CV Bridge
        self.subscription = self.create_subscription(
            Image,
            image_topic,
            self.image_callback,
            10  # QoS profile depth
        )
        self.bridge = CvBridge()
        
        # Data buffer
        self.image_buffer = []
        self.batch_counter = 0

        self.get_logger().info(f"VAE Trainer Node started. Device: {device}")
        self.get_logger().info(f"Subscribing to: {image_topic}")
        self.get_logger().info(f"Training with: latent_dim={latent_dim}, batch_size={self.batch_size}")
        self.get_logger().info(f"Target image shape: {self.image_shape}")

    def image_callback(self, msg):
        depth_array = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

        # Replace NaNs with a valid number (e.g., 0)
        depth_array = np.nan_to_num(depth_array, nan=5.0)

        normalized_depth_image_display = cv2.normalize(depth_array, None, 0.0, 1.0, cv2.NORM_MINMAX, dtype=cv2.CV_32F)
        resized_image_hwc = cv2.resize(normalized_depth_image_display, (32,32), interpolation=cv2.INTER_LINEAR)
        
        # Add clip here: Resize can introduce interpolation artifacts outside [0, 1]
        resized_image_hwc = np.clip(resized_image_hwc, 0.0, 1.0)
        
        resized_image_cwh = resized_image_hwc[np.newaxis, :, :] # Add channel dimension
        # This transpose was likely an error and would swap H and W.
        # resized_image_cwh = np.transpose(resized_image_cwh, (0, 2, 1))
        
        # Add to buffer
        self.image_buffer.append(resized_image_cwh)
        
        # Check if buffer is full
        if len(self.image_buffer) >= self.batch_size:
            self.train_batch()
            # Clear buffer
            self.image_buffer = []

    def train_batch(self):
        self.batch_counter += 1
        
        # Prepare batch
        try:
            batch_data = np.array(self.image_buffer[:self.batch_size])
            images_tensor = torch.FloatTensor(batch_data).to(device)
        except Exception as e:
            self.get_logger().error(f"Failed to create batch: {e}")
            return

        # --- VAE Training Step ---
        
        # Forward pass
        reconstruction, mu, log_var = self.vae(images_tensor)
        
        # Calculate loss
        loss = self.vae.loss_function(reconstruction, images_tensor, mu, log_var)
        avg_loss = loss.item() / self.batch_size # Average loss per image
        
        # Backward pass and optimize
        self.optimizer.zero_grad()
        loss.backward()
        self.optimizer.step()
        
        if self.batch_counter % 10 == 0: # Log every 10 batches
            self.get_logger().info(f"Batch {self.batch_counter}: Avg. VAE Loss = {avg_loss:.4f}")

        # Save model weights periodically
        if self.batch_counter % self.save_every_n == 0:
            try:
                # We only save the encoder, as that's what the RL agent needs
                torch.save(self.vae.encoder.state_dict(), self.save_path)
                self.get_logger().info(f"Saved VAE encoder weights to {self.save_path}")
            except Exception as e:
                self.get_logger().error(f"Failed to save model: {e}")


def main(args=None):
    rclpy.init(args=args)
    try:
        node = VAETrainerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Unhandled exception: {e}")
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()

if __name__ == '__main__':
    main()