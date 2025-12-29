"""
Scikit-learn Neural Network with Reinforcement Learning Trainer

Uses sklearn's MLPRegressor (2 hidden layers, 10 nodes each) with Q-learning
to train a controller for the inverted pendulum balance task.

Inputs (4):
  - Angle from upright (normalized: 180° = 0, 0°/360° = ±1)
  - Angular velocity (normalized)
  - Cart position (normalized)
  - Cart velocity (normalized)

Output (1):
  - Cart acceleration command (-1 to 1, scaled to max acceleration)

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
from simulator import PendulumSimulator
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
HIDDEN_LAYER_SIZES = (10, 10)  # 2 hidden layers of 10 nodes each


class QLearningAgent:
    """Q-Learning agent using sklearn MLPRegressor"""
    
    def __init__(self, state_size=4, action_size=1):
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


def compute_reward(state, angle_deg, prev_angle_deg):
    """Compute reward for current state"""
    angle_from_up = abs(normalize_angle(angle_deg))
    
    # Reward for being near upright
    if angle_from_up < 0.167:  # Within 30° of upright
        reward = 1.0
    else:
        reward = -0.1
    
    # Penalty for falling
    if angle_from_up > 0.25:  # Past 45°
        reward = -10.0
    
    # Penalty for hitting limits
    if state['limit_left'] or state['limit_right']:
        reward = -50.0
    
    return reward


def train_episode(agent, episode_num):
    """Train for one episode"""
    sim_config = create_fast_simulator()
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
    prev_angle_deg = math.degrees(state['pendulum_angle']) % 360
    
    for step in range(SIMULATION_STEPS):
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        
        # Prepare state vector
        state_vec = [
            normalize_angle(angle_deg),
            state['pendulum_velocity'] / 1000.0,
            state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            state['cart_velocity'] / MAX_SPEED
        ]
        
        # Get action
        action = agent.get_action(state_vec, training=True)
        
        # Convert action to acceleration
        max_accel = sim_config.motor_accel
        acceleration = action * max_accel
        
        # Apply action
        sim.set_target_acceleration(acceleration)
        sim.step(EVAL_DT)
        
        # Get next state
        next_state = sim.get_state()
        next_angle_deg = math.degrees(next_state['pendulum_angle']) % 360
        
        # Compute reward
        reward = compute_reward(next_state, next_angle_deg, prev_angle_deg)
        total_reward += reward
        
        # Check if done
        done = (next_state['limit_left'] or next_state['limit_right'] or 
                abs(normalize_angle(next_angle_deg)) > 0.25)
        
        # Prepare next state vector
        next_state_vec = [
            normalize_angle(next_angle_deg),
            next_state['pendulum_velocity'] / 1000.0,
            next_state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            next_state['cart_velocity'] / MAX_SPEED
        ]
        
        # Store experience
        agent.remember(state_vec, action, reward, next_state_vec, done)
        
        # Train on batch
        agent.replay()
        
        prev_angle_deg = next_angle_deg
        
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
    
    agent = QLearningAgent(state_size=4, action_size=1)
    
    print("Starting Q-Learning training with sklearn MLPRegressor...")
    print(f"Network architecture: 4 inputs -> {HIDDEN_LAYER_SIZES} -> 1 output")
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
        
        if (episode + 1) % 10 == 0:
            print(f"Episode {episode + 1}, Reward: {reward:.1f}, Steps: {steps}, "
                  f"Best: {best_reward:.1f}, Exploration: {agent.exploration_rate:.3f}")
        
        episode += 1
        
        # Check if we've reached the episode limit
        if episode >= EPISODES:
            print(f"Reached episode limit ({EPISODES}), continuing indefinitely...")
            EPISODES = float('inf')  # Continue indefinitely
    
    print(f"Training stopped. Best reward: {best_reward:.1f}")
    agent.save(MODEL_PATH, SCALER_PATH)


def test():
    """Test the trained model"""
    agent = QLearningAgent(state_size=4, action_size=1)
    
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
        
        # Prepare state vector
        state_vec = [
            normalize_angle(angle_deg),
            state['pendulum_velocity'] / 1000.0,
            state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            state['cart_velocity'] / MAX_SPEED
        ]
        
        # Get action (no exploration during test)
        action = agent.get_action(state_vec, training=False)
        
        # Convert to acceleration
        max_accel = sim_config.motor_accel
        acceleration = action * max_accel
        
        # Apply action
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

