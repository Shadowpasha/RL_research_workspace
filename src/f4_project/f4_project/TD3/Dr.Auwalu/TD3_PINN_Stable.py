import copy
import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")

# --- Actor and Critic Networks (Unchanged) ---
# These are generic and will adapt to the state_dim and action_dim
# provided during instantiation.

class Actor(nn.Module):
    def __init__(self, state_dim, action_dim, max_action):
        super(Actor, self).__init__()
        self.l1 = nn.Linear(state_dim, 800)
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, action_dim)
        self.max_action = max_action

    def forward(self, state):
        a = F.relu(self.l1(state))
        a = F.relu(self.l2(a))
        a = torch.tanh(self.l3(a))
        return self.max_action * a


class Critic(nn.Module):
    def __init__(self, state_dim, action_dim):
        super(Critic, self).__init__()
        # Q1
        self.l1 = nn.Linear(state_dim + action_dim, 800)
        self.l2 = nn.Linear(800, 600)
        self.l3 = nn.Linear(600, 1)
        # Q2
        self.l4 = nn.Linear(state_dim + action_dim, 800)
        self.l5 = nn.Linear(800, 600)
        self.l6 = nn.Linear(600, 1)

    def forward(self, state, action):
        sa = torch.cat([state, action], dim=1)
        q1 = F.relu(self.l1(sa)); q1 = F.relu(self.l2(q1)); q1 = self.l3(q1)
        q2 = F.relu(self.l4(sa)); q2 = F.relu(self.l5(q2)); q2 = self.l6(q2)
        return q1, q2

    def Q1(self, state, action):
        sa = torch.cat([state, action], dim=1)
        q1 = F.relu(self.l1(sa)); q1 = F.relu(self.l2(q1)); q1 = self.l3(q1)
        return q1


class TD3_PINN_Stable(object):
    """
    Stable TD3 + PINN hybrid for a 3D Drone with high-level 2D commands.
    Expected state_dim = 34
    Expected action_dim = 2 (cmd_vel_fwd, cmd_vel_yaw)

    State definition (last 10 elements):
    [24]: prev_action[0] (cmd_vel_fwd)
    [25]: prev_action[1] (cmd_vel_yaw)
    [26]: distance_to_goal
    [27]: heading_error (goal_heading - true_yaw)
    [28]: vel.linear.x (ASSUMED to be BODY-FRAME forward velocity)
    [29]: vel.linear.y (ASSUMED to be BODY-FRAME lateral velocity)
    [30]: pose.position.x (world frame)
    [31]: pose.position.y (world frame)
    [32]: roll
    [33]: pitch

    After instantiation, set:
        agent.state_mean, agent.state_std, agent.action_mean, agent.action_std
    so PINN uses physical (denormalized) values.
    """

    def __init__(
        self,
        state_dim,      # Should be 34
        action_dim,     # Should be 2
        max_action,
        discount=0.99,
        tau=0.005,
        policy_noise=0.2,
        noise_clip=0.5,
        policy_freq=2,
        # PINN options
        enable_pinn=True,
        lambda_pinn=1e-3,
        pinn_every=5,           # apply PINN loss every N critic updates
        dt=0.02,                # Timestep (e.g., 20ms)
        tau_v=0.5,              # Time constant for forward velocity (s)
        tau_w=0.3,              # Time constant for yaw velocity (s)
        critic_lr=1e-4,         # reduced critic LR for stability
        actor_lr=3e-4
    ):
        # --- Standard TD3 Init ---
        self.actor = Actor(state_dim, action_dim, max_action).to(device)
        self.actor_target = copy.deepcopy(self.actor)
        self.actor_optimizer = torch.optim.Adam(self.actor.parameters(), lr=actor_lr)

        self.critic = Critic(state_dim, action_dim).to(device)
        self.critic_target = copy.deepcopy(self.critic)
        self.critic_optimizer = torch.optim.Adam(self.critic.parameters(), lr=critic_lr)

        self.max_action = max_action
        self.discount = discount
        self.tau = tau
        self.policy_noise = policy_noise
        self.noise_clip = noise_clip
        self.policy_freq = policy_freq
        self.total_it = 0

        # --- PINN settings (Updated for 2D Ground Robot) ---
        self.enable_pinn = enable_pinn
        self.lambda_pinn = lambda_pinn
        self.pinn_every = pinn_every
        self.dt = float(dt)

        # Physics parameters for the ground robot
        # Time constant for 1st-order model of forward velocity
        self.tau_v = torch.tensor(tau_v, dtype=torch.float32, device=device)
        # Time constant for 1st-order model of yaw velocity
        # Note: We don't use this if we model yaw_dot = cmd_w
        self.tau_w = torch.tensor(tau_w, dtype=torch.float32, device=device)


        # denormalization placeholders (set these from main script)
        self.state_mean = np.zeros(state_dim, dtype=np.float32)
        self.state_std  = np.ones(state_dim, dtype=np.float32)
        self.action_mean = np.zeros(action_dim, dtype=np.float32)
        self.action_std  = np.ones(action_dim, dtype=np.float32)


    def select_action(self, state_np):
        state = torch.FloatTensor(state_np.reshape(1, -1)).to(device)
        action = self.actor(state)
        return action.cpu().data.numpy().flatten()


    def _denormalize_tensors(self, state, next_state, action):
        """
        state, next_state, action are torch tensors in normalized space;
        this function returns denormalized (physical) tensors for PINN computation.
        If inputs are already physical, this will act as identity if means/std are zeros/ones.
        """
        # Ensure stats are torch tensors on the correct device
        if not isinstance(self.state_mean, torch.Tensor):
            self.state_mean = torch.tensor(self.state_mean, dtype=torch.float32, device=device)
        if not isinstance(self.state_std, torch.Tensor):
            self.state_std = torch.tensor(self.state_std, dtype=torch.float32, device=device)
        if not isinstance(self.action_mean, torch.Tensor):
            self.action_mean = torch.tensor(self.action_mean, dtype=torch.float32, device=device)
        if not isinstance(self.action_std, torch.Tensor):
            self.action_std = torch.tensor(self.action_std, dtype=torch.float32, device=device)

        s_phys = state * self.state_std + self.state_mean
        s2_phys = next_state * self.state_std + self.state_mean
        a_phys = action * self.action_std + self.action_mean
        return s_phys, s2_phys, a_phys


    def physics_loss_batch(self, state_phys, next_state_phys, action_phys, ema_alpha=1, normalize_residuals=True):
        """
        Calculates physics-based residuals for a 3D drone
        using high-level 2D commands.

        ASSUMPTIONS:
        1. state_phys[..., 28] is BODY-FRAME forward velocity (vx_body).
        2. state_phys[..., 29] is BODY-FRAME lateral velocity (vy_body).
        3. state_phys[..., 32] is BODY-FRAME roll angle.
        4. state_phys[..., 27] is heading_error = goal_heading - true_yaw.
        5. goal_heading is constant over the timestep dt.
        6. action_phys[..., 0] is the commanded forward velocity (v_cmd).
        7. action_phys[..., 1] is the commanded yaw velocity (w_cmd).
        """
        B = state_phys.shape[0]
        dt = float(self.dt)

        # --- Extract states and actions ---
        v_cmd_t = action_phys[:, 0]
        w_cmd_t = action_phys[:, 1]

        vx_t = state_phys[:, 28]
        vx_next = next_state_phys[:, 28]

        vy_t = state_phys[:, 29]
        
        roll_t = state_phys[:, 32]

        heading_err_t = state_phys[:, 27]
        heading_err_next = next_state_phys[:, 27]


        # --- Residual 1: Forward Velocity Dynamics (1st-order lag) ---
        # Measured acceleration (body-frame)
        vx_dot_measured = (vx_next - vx_t) / dt
        # Modeled acceleration (body-frame)
        vx_dot_model = (v_cmd_t - vx_t) / self.tau_v
        
        r_fwd_vel = vx_dot_measured - vx_dot_model


        # --- Residual 2: Yaw Velocity Dynamics (0-order model) ---
        # Measured yaw rate (from change in heading error)
        # yaw_dot = d/dt(true_yaw) = d/dt(goal - heading_err) = - d/dt(heading_err)
        yaw_dot_measured = (heading_err_t - heading_err_next) / dt
        
        # Modeled yaw rate (simplest model: command = actual)
        # A 1st-order model would be:
        # yaw_dot_model = (w_cmd_t - yaw_dot_measured) / self.tau_w
        # But this requires yaw_dot_measured to be a state.
        # We use the 0-order model for simplicity, as yaw tracking is usually fast.
        yaw_dot_model = w_cmd_t
        
        r_yaw_vel = yaw_dot_measured - yaw_dot_model

        
        # --- Residual 3: Lateral Velocity (Non-holonomic style constraint) ---
        # The lateral body-frame velocity should be zero if not commanded.
        r_lat_vel = vy_t

        # --- Residual 4: Roll Angle (Constraint) ---
        # Roll should be zero if no lateral motion is commanded.
        r_roll = roll_t


        # --- EMA smoothing (optional, same as before) ---
        if ema_alpha is not None and 0.0 < ema_alpha < 1.0:
            r_fwd_vel = ema_alpha * r_fwd_vel + (1 - ema_alpha) * r_fwd_vel.detach()
            r_yaw_vel = ema_alpha * r_yaw_vel + (1 - ema_alpha) * r_yaw_vel.detach()
            r_lat_vel = ema_alpha * r_lat_vel + (1 - ema_alpha) * r_lat_vel.detach()
            r_roll    = ema_alpha * r_roll    + (1 - ema_alpha) * r_roll.detach()


        # --- Normalize residuals (optional, same as before) ---
        if normalize_residuals:
            s_fwd = torch.std(r_fwd_vel) + 1e-8
            s_yaw = torch.std(r_yaw_vel) + 1e-8
            s_lat = torch.std(r_lat_vel) + 1e-8
            s_roll = torch.std(r_roll) + 1e-8

            r_fwd_vel = r_fwd_vel / s_fwd
            r_yaw_vel = r_yaw_vel / s_yaw
            r_lat_vel = r_lat_vel / s_lat
            r_roll    = r_roll    / s_roll

        # --- Compute Losses ---
        loss_fwd_vel = torch.mean(r_fwd_vel**2)
        loss_yaw_vel = torch.mean(r_yaw_vel**2)
        loss_lat_vel = torch.mean(r_lat_vel**2)
        loss_roll    = torch.mean(r_roll**2)

        pinn_loss = loss_fwd_vel + loss_yaw_vel + loss_lat_vel + loss_roll

        stats = {
            'loss_fwd_vel': loss_fwd_vel.item(),
            'loss_yaw_vel': loss_yaw_vel.item(),
            'loss_lat_vel': loss_lat_vel.item(),
            'loss_roll':    loss_roll.item()
        }
        return pinn_loss, stats


    def train(self, replay_buffer, batch_size=1024):
        """
        One training iteration (sample batch, update critic and occasionally actor/targets).
        This method applies PINN loss only every self.pinn_every updates (to reduce noise).
        """
        self.total_it += 1

        # sample
        state, action, next_state, reward, not_done = replay_buffer.sample(batch_size)
        
        # Move to device
        state = state.to(device)
        action = action.to(device)
        next_state = next_state.to(device)
        reward = reward.to(device)
        not_done = not_done.to(device)


        with torch.no_grad():
            noise = (torch.randn_like(action) * self.policy_noise).clamp(-self.noise_clip, self.noise_clip)
            next_action = (self.actor_target(next_state) + noise).clamp(-self.max_action, self.max_action)

            target_Q1, target_Q2 = self.critic_target(next_state, next_action)
            target_Q = torch.min(target_Q1, target_Q2)
            target_Q = reward + not_done * self.discount * target_Q

        current_Q1, current_Q2 = self.critic(state, action)
        critic_loss = F.mse_loss(current_Q1, target_Q) + F.mse_loss(current_Q2, target_Q)

        pinn_loss_val = torch.tensor(0.0, device=device)
        pinn_stats = None
        # apply PINN every pinn_every iterations
        if self.enable_pinn and (self.total_it % self.pinn_every == 0):
            # denormalize to physical scale
            s_phys, s2_phys, a_phys = self._denormalize_tensors(state, next_state, action)
            
            # Use the new physics loss function
            pinn_loss_val, pinn_stats = self.physics_loss_batch(s_phys, s2_phys, a_phys,
                                                                ema_alpha=1, normalize_residuals=True)
            
            critic_loss = critic_loss + self.lambda_pinn * pinn_loss_val

        # critic step
        self.critic_optimizer.zero_grad()
        critic_loss.backward()
        self.critic_optimizer.step()

        Q_value = torch.mean(current_Q1).detach() # Use current Q1 for logging

        # delayed policy updates
        if self.total_it % self.policy_freq == 0:
            actor_loss = -self.critic.Q1(state, self.actor(state)).mean()
            self.actor_optimizer.zero_grad()
            actor_loss.backward()
            self.actor_optimizer.step()

            # update targets
            for param, target_param in zip(self.critic.parameters(), self.critic_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)
            for param, target_param in zip(self.actor.parameters(), self.actor_target.parameters()):
                target_param.data.copy_(self.tau * param.data + (1 - self.tau) * target_param.data)

        out = {
            'Q_value': Q_value.item() if isinstance(Q_value, torch.Tensor) else float(Q_value),
            'critic_loss': critic_loss.item()
        }
        if self.enable_pinn and (pinn_stats is not None):
            out['pinn_loss'] = pinn_loss_val.item()
            out.update(pinn_stats) # Adds loss_fwd_vel, loss_yaw_vel, etc.
        else:
            out['pinn_loss'] = 0.0
        return out


    def save(self, prefix):
        torch.save(self.critic.state_dict(), prefix + "_critic.pth")
        torch.save(self.critic_optimizer.state_dict(), prefix + "_critic_optimizer.pth")
        torch.save(self.actor.state_dict(), prefix + "_actor.pth")
        torch.save(self.actor_optimizer.state_dict(), prefix + "_actor_optimizer.pth")


    def load(self, prefix):
        self.critic.load_state_dict(torch.load(prefix + "_critic.pth", map_location=device))
        self.critic_optimizer.load_state_dict(torch.load(prefix + "_critic_optimizer.pth", map_location=device))
        self.critic_target = copy.deepcopy(self.critic)
        self.actor.load_state_dict(torch.load(prefix + "_actor.pth", map_location=device))
        self.actor_optimizer.load_state_dict(torch.load(prefix + "_actor_optimizer.pth", map_location=device))
        self.actor_target = copy.deepcopy(self.actor)

