"""
Scikit-learn Neural Network with Reinforcement Learning Trainer

Uses sklearn's MLPRegressor (2 hidden layers, 10 nodes each) with Q-learning
to train a controller for the inverted pendulum balance task.

Inputs (5):
  - sin(θ) - sine of pendulum angle (prevents angle wrap discontinuity)
  - cos(θ) - cosine of pendulum angle (prevents angle wrap discontinuity)
  - Angular velocity (normalized)
  - Cart position (normalized)
  - Cart velocity (normalized)

Output (1):
  - Cart acceleration command (-1 to 1, scaled to max acceleration)
  - Controller uses acceleration-only control (no velocity commands)

Usage:
  python sklearn_rl_trainer.py              # Train new network
  python sklearn_rl_trainer.py --test       # Test trained network
"""

import os
import sys
import math
import json
import pickle
import random
import time
import numpy as np
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from simulator import PendulumSimulator, SimulatorConfig
from trainer_utils import create_fast_simulator, normalize_angle, load_training_params, send_training_update

# Paths
MODEL_PATH = os.path.join(os.path.dirname(__file__), 'sklearn_rl_model.pkl')
SCALER_PATH = os.path.join(os.path.dirname(__file__), 'sklearn_rl_scaler.pkl')
TRAINING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
STOP_TRAINING_FILE = os.path.join(os.path.dirname(__file__), 'stop_training.flag')

# Training parameters (defaults, can be overridden by config)
MAX_SPEED = 9000
SIMULATION_STEPS = 5000
EPISODES = 1000
EVAL_DT = 0.02  # Evaluation timestep (50Hz)

# Q-Learning parameters
LEARNING_RATE = 0.001
DISCOUNT_FACTOR = 0.95
EXPLORATION_RATE = 0.1
EXPLORATION_DECAY = 0.995
MIN_EXPLORATION = 0.01

# Network architecture
HIDDEN_LAYER_SIZES = (10)  # 1 hidden layers of 10 nodes each

# Progressive training parameters
PROGRESSIVE_TRAINING_EPISODES = 100000  # Episodes to gradually increase gravity
INITIAL_GRAVITY = 2.0  # Start with lower gravity (m/s²)
FINAL_GRAVITY = 9.81  # Final gravity (m/s²)
INITIAL_DAMPING = 0.5  # Start with higher damping
FINAL_DAMPING = 0.1  # Final damping (matches create_fast_simulator)


class QLearningAgent:
    """Q-Learning agent using sklearn MLPRegressor"""
    
    def __init__(self, state_size=5, action_size=1):
        self.state_size = state_size
        self.action_size = action_size
        self.exploration_rate = EXPLORATION_RATE
        
        # Create neural network (2 hidden layers, 10 nodes each)
        self.q_network = MLPRegressor(
            hidden_layer_sizes=HIDDEN_LAYER_SIZES,
            activation='tanh',
            solver='adam',
            alpha=0.001,  # L2 regularization
            learning_rate='adaptive',
            max_iter=1,  # We'll train incrementally
            warm_start=True,  # Allow incremental training
            random_state=42
        )
        
        # Initialize with dummy data
        dummy_state = np.zeros((1, state_size))
        dummy_q = np.zeros((1, action_size))
        self.q_network.fit(dummy_state, dummy_q)
        
        # Scaler for state normalization
        self.scaler = StandardScaler()
        self.scaler.fit(dummy_state)
        
        # Experience replay buffer
        self.memory = []
        self.memory_size = 10000
        self.batch_size = 32
    
    def get_action(self, state, training=True):
        """Get action using epsilon-greedy policy"""
        state_scaled = self.scaler.transform([state])
        
        if training and random.random() < self.exploration_rate:
            # Explore: random action
            return random.uniform(-1.0, 1.0)
        else:
            # Exploit: use Q-network
            q_value = self.q_network.predict(state_scaled)
            # predict() returns 1D array, extract first (and only) value
            if isinstance(q_value, np.ndarray):
                action = float(q_value.flat[0])  # Use flat to get scalar value
            else:
                action = float(q_value)
            return np.clip(action, -1.0, 1.0)
    
    def remember(self, state, action, reward, next_state, done):
        """Store experience in replay buffer"""
        self.memory.append((state, action, reward, next_state, done))
        if len(self.memory) > self.memory_size:
            self.memory.pop(0)
    
    def replay(self):
        """Train Q-network on a batch of experiences"""
        if len(self.memory) < self.batch_size:
            return
        
        # Sample batch
        batch = random.sample(self.memory, self.batch_size)
        states = np.array([e[0] for e in batch])
        actions = np.array([e[1] for e in batch])
        rewards = np.array([e[2] for e in batch])
        next_states = np.array([e[3] for e in batch])
        dones = np.array([e[4] for e in batch])
        
        # Scale states
        states_scaled = self.scaler.transform(states)
        next_states_scaled = self.scaler.transform(next_states)
        
        # Compute target Q-values
        current_q = self.q_network.predict(states_scaled)
        next_q = self.q_network.predict(next_states_scaled)
        
        # Compute target Q-values (ensure 1D arrays)
        current_q = current_q.flatten()
        next_q = next_q.flatten()
        
        # Build targets array
        targets = current_q.copy()
        for i in range(len(batch)):
            if dones[i]:
                targets[i] = rewards[i]
            else:
                targets[i] = rewards[i] + DISCOUNT_FACTOR * next_q[i]
        
        # Update Q-network (targets must be 1D for sklearn)
        self.q_network.partial_fit(states_scaled, targets)
        
        # Decay exploration
        if self.exploration_rate > MIN_EXPLORATION:
            self.exploration_rate *= EXPLORATION_DECAY
    
    def save(self, model_path, scaler_path):
        """Save model and scaler"""
        with open(model_path, 'wb') as f:
            pickle.dump(self.q_network, f)
        with open(scaler_path, 'wb') as f:
            pickle.dump(self.scaler, f)
        print(f"Saved model to {model_path}")
    
    def load(self, model_path, scaler_path):
        """Load model and scaler"""
        if os.path.exists(model_path) and os.path.exists(scaler_path):
            with open(model_path, 'rb') as f:
                self.q_network = pickle.load(f)
            with open(scaler_path, 'rb') as f:
                self.scaler = pickle.load(f)
            print(f"Loaded model from {model_path}")
            return True
        return False


def get_progressive_physics(episode_num):
    """
    Calculate progressive gravity and damping values based on episode number.
    
    For episodes 0 to PROGRESSIVE_TRAINING_EPISODES:
    - Gravity: linearly increases from INITIAL_GRAVITY to FINAL_GRAVITY
    - Damping: linearly decreases from INITIAL_DAMPING to FINAL_DAMPING
    
    After PROGRESSIVE_TRAINING_EPISODES, uses final values.
    
    Args:
        episode_num: Current episode number (0-indexed)
    
    Returns:
        Tuple of (gravity, damping)
    """
    if episode_num >= PROGRESSIVE_TRAINING_EPISODES:
        return FINAL_GRAVITY, FINAL_DAMPING
    
    # Linear interpolation
    progress = episode_num / PROGRESSIVE_TRAINING_EPISODES
    
    gravity = INITIAL_GRAVITY + (FINAL_GRAVITY - INITIAL_GRAVITY) * progress
    damping = INITIAL_DAMPING + (FINAL_DAMPING - INITIAL_DAMPING) * progress
    
    return gravity, damping


def compute_reward(state, angle_rad, angular_velocity, cart_position, cart_velocity, acceleration, rail_length, max_accel):
    """
    Compute reward for balancing task.
    
    Reward structure:
    - Upright term: -cos(θ) (max at θ=π/180°, upright position)
    - Penalize angular velocity: -k1 * θ̇²
    - Penalize cart position: -k2 * x²
    - Penalize cart velocity: -k3 * ẋ²
    - Penalize aggressive control: -k4 * a²
    - Big terminal penalty if falling past angle threshold
    
    Args:
        state: Simulator state dict
        angle_rad: Pendulum angle in radians (0 = DOWN, π = UP)
        angular_velocity: Angular velocity in rad/s
        cart_position: Cart position in steps
        cart_velocity: Cart velocity in steps/s
        acceleration: Applied acceleration in steps/s²
        rail_length: Rail length in steps
        max_accel: Maximum acceleration in steps/s² (for normalization)
    
    Returns:
        Reward value
    """
    # Reward coefficients
    k1 = 0.01   # Angular velocity penalty
    k2 = 0.001  # Cart position penalty
    k3 = 0.01   # Cart velocity penalty
    k4 = 0.001  # Control action penalty
    
    # Upright term: -cos(θ) (max when θ=π, i.e., upright)
    # In simulator: θ=0 is DOWN, θ=π is UP (upright)
    upright_term = -math.cos(angle_rad)
    
    # Normalize values for penalty terms
    # Angular velocity: already in rad/s
    angular_vel_sq = angular_velocity ** 2
    
    # Cart position: normalize to [-1, 1] range (center = 0)
    cart_pos_normalized = cart_position / (rail_length / 2) if rail_length > 0 else 0
    cart_pos_sq = cart_pos_normalized ** 2
    
    # Cart velocity: normalize by max speed
    cart_vel_normalized = cart_velocity / MAX_SPEED
    cart_vel_sq = cart_vel_normalized ** 2
    
    # Acceleration: normalize by max acceleration
    accel_normalized = acceleration / max_accel
    accel_sq = accel_normalized ** 2
    
    # Compute base reward
    reward = upright_term - k1 * angular_vel_sq - k2 * cart_pos_sq - k3 * cart_vel_sq - k4 * accel_sq
    
    # Terminal penalty: big penalty if falling past threshold (20° = ~0.35 radians from upright)
    angle_from_up_rad = abs(angle_rad - math.pi)  # Distance from π (upright)
    if angle_from_up_rad > math.radians(20):  # Past 20° from upright
        reward = -50.0  # Big terminal penalty
    
    # Penalty for hitting rail limits
    if state['limit_left'] or state['limit_right']:
        reward = -50.0
    
    return reward


def train_episode(agent, episode_num):
    """Train for one episode"""
    # Get progressive physics values
    gravity, damping = get_progressive_physics(episode_num)
    
    # Create simulator config with progressive values
    sim_config = SimulatorConfig(
        rail_length_steps=7500,
        cart_mass=0.5,
        motor_accel=500000,
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=gravity,
        damping=damping,
    )
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Start pendulum at 180° (upright) with small perturbation
    perturbation = random.uniform(-10, 10)
    # Start with initial cart velocity to prevent "no action" from being a good strategy
    initial_cart_velocity = random.uniform(-MAX_SPEED * 0.3, MAX_SPEED * 0.3)
    sim.set_state(
        cart_position=0.0,
        cart_velocity=initial_cart_velocity,
        pendulum_angle=math.radians(180 + perturbation),
        pendulum_velocity=random.uniform(-0.5, 0.5)
    )
    
    # Get rail limits
    state = sim.get_state()
    rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    total_reward = 0.0
    
    for step in range(SIMULATION_STEPS):
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        angle_rad = state['pendulum_angle']  # Already in radians
        
        # Prepare state vector using sin(θ) and cos(θ) to prevent angle wrap discontinuity
        state_vec = [
            math.sin(angle_rad),
            math.cos(angle_rad),
            state['pendulum_velocity'] / 1000.0,
            state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            state['cart_velocity'] / MAX_SPEED
        ]
        
        # Get action (output is normalized acceleration command in range [-1, 1])
        action = agent.get_action(state_vec, training=True)
        
        # Convert normalized action to actual acceleration
        # Controller uses acceleration-only control (no velocity commands)
        max_accel = sim_config.motor_accel
        acceleration = action * max_accel
        
        # Apply acceleration command directly to motor
        sim.set_target_acceleration(acceleration)
        sim.step(EVAL_DT)
        
        # Get next state
        next_state = sim.get_state()
        next_angle_deg = math.degrees(next_state['pendulum_angle']) % 360
        next_angle_rad = next_state['pendulum_angle']  # Already in radians
        
        # Compute reward using new reward structure
        reward = compute_reward(
            next_state,
            next_angle_rad,
            next_state['pendulum_velocity'],
            next_state['cart_position'],
            next_state['cart_velocity'],
            acceleration,
            rail_length,
            max_accel
        )
        total_reward += reward
        
        # Check if done: |θ - π| > 20° or |x| > rail_limit
        angle_from_up_rad = abs(next_angle_rad - math.pi)
        cart_pos_normalized = abs(next_state['cart_position'] / (rail_length / 2)) if rail_length > 0 else 0
        done = (next_state['limit_left'] or next_state['limit_right'] or 
                angle_from_up_rad > math.radians(20) or
                cart_pos_normalized > 1.0)
        
        # Prepare next state vector using sin(θ) and cos(θ) to prevent angle wrap discontinuity
        next_state_vec = [
            math.sin(next_angle_rad),
            math.cos(next_angle_rad),
            next_state['pendulum_velocity'] / 1000.0,
            next_state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            next_state['cart_velocity'] / MAX_SPEED
        ]
        
        # Store experience
        agent.remember(state_vec, action, reward, next_state_vec, done)
        
        # Train on batch
        agent.replay()
        
        if done:
            break
    
    sim.close()
    return total_reward, step + 1


def load_sklearn_config():
    """Load sklearn training config from JSON file"""
    global MAX_SPEED, SIMULATION_STEPS, EPISODES
    
    try:
        if os.path.exists(TRAINING_CONFIG_FILE):
            with open(TRAINING_CONFIG_FILE, 'r') as f:
                config = json.load(f)
                sklearn_config = config.get('sklearn', {})
                MAX_SPEED = sklearn_config.get('max_speed', MAX_SPEED)
                SIMULATION_STEPS = sklearn_config.get('sim_steps', SIMULATION_STEPS)
                EPISODES = sklearn_config.get('episodes', EPISODES)
    except Exception as e:
        print(f"Warning: Could not load sklearn config: {e}")

def train():
    """Train the Q-learning agent"""
    global MAX_SPEED, SIMULATION_STEPS, EPISODES
    
    # Load config
    load_sklearn_config()
    
    # Clear stop flag
    if os.path.exists(STOP_TRAINING_FILE):
        os.remove(STOP_TRAINING_FILE)
    
    agent = QLearningAgent(state_size=5, action_size=1)
    
    print("Starting Q-Learning training with sklearn MLPRegressor...")
    print(f"Network architecture: 5 inputs (sin(θ), cos(θ), angular_vel, cart_pos, cart_vel) -> {HIDDEN_LAYER_SIZES} -> 1 output")
    print(f"Episodes: {EPISODES}, Max Speed: {MAX_SPEED}, Sim Steps: {SIMULATION_STEPS}")
    
    best_reward = float('-inf')
    episode = 0
    
    while True:
        # Check for stop flag
        if os.path.exists(STOP_TRAINING_FILE):
            print("Stop flag detected, stopping training...")
            break
        
        reward, steps = train_episode(agent, episode)
        
        if reward > best_reward:
            best_reward = reward
            agent.save(MODEL_PATH, SCALER_PATH)
            print(f"New best reward: {best_reward:.1f}, saved model")
        
        # Send training update
        send_training_update({
            'trainer': 'sklearn',
            'episode': episode + 1,
            'best_reward': best_reward,
            'last_reward': reward,
            'exploration_rate': agent.exploration_rate,
            'steps': steps
        })
        
        # Get current physics values for display
        current_gravity, current_damping = get_progressive_physics(episode)
        
        if (episode + 1) % 10 == 0:
            print(f"Episode {episode + 1}, Reward: {reward:.1f}, Steps: {steps}, "
                  f"Best: {best_reward:.1f}, Exploration: {agent.exploration_rate:.3f}, "
                  f"Gravity: {current_gravity:.2f}, Damping: {current_damping:.3f}")
        
        episode += 1
        
        # Check if we've reached the episode limit
        if episode >= EPISODES:
            print(f"Reached episode limit ({EPISODES}), continuing indefinitely...")
            EPISODES = float('inf')  # Continue indefinitely
    
    print(f"Training stopped. Best reward: {best_reward:.1f}")
    agent.save(MODEL_PATH, SCALER_PATH)


def test():
    """Test the trained model"""
    agent = QLearningAgent(state_size=5, action_size=1)
    
    if not agent.load(MODEL_PATH, SCALER_PATH):
        print("No trained model found. Run training first.")
        return
    
    sim_config = create_fast_simulator()
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Start pendulum at 180° (upright) with small perturbation
    sim.set_state(
        cart_position=0.0,
        cart_velocity=0.0,
        pendulum_angle=math.radians(180 + 5),
        pendulum_velocity=0.0
    )
    
    state = sim.get_state()
    rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    print("\nRunning test simulation...")
    print("Time(s) | Angle(°) | Acceleration")
    print("-" * 40)
    
    for step in range(SIMULATION_STEPS * 2):
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        angle_rad = state['pendulum_angle']  # Already in radians
        
        # Prepare state vector using sin(θ) and cos(θ) to prevent angle wrap discontinuity
        state_vec = [
            math.sin(angle_rad),
            math.cos(angle_rad),
            state['pendulum_velocity'] / 1000.0,
            state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            state['cart_velocity'] / MAX_SPEED
        ]
        
        # Get action (output is normalized acceleration command in range [-1, 1])
        # Controller uses acceleration-only control (no velocity commands)
        action = agent.get_action(state_vec, training=False)
        
        # Convert normalized action to actual acceleration
        max_accel = sim_config.motor_accel
        acceleration = action * max_accel
        
        # Apply acceleration command directly to motor
        sim.set_target_acceleration(acceleration)
        sim.step(EVAL_DT)
        
        if step % 25 == 0:
            time_s = step * EVAL_DT
            print(f"{time_s:7.1f} | {angle_deg:8.1f} | {acceleration:11.0f}")
    
    sim.close()


if __name__ == "__main__":
    if "--test" in sys.argv:
        test()
    else:
        train()

