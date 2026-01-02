"""
EvoTorch Neural Network Balance Trainer

Uses PyTorch neural network (2 hidden layers, 10 nodes each) with EvoTorch
evolutionary algorithm to train a controller for the inverted pendulum balance task.

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
  python evotorch_balance_trainer.py              # Train new network
  python evotorch_balance_trainer.py --test       # Test trained network
"""

import os
import sys
import math
import json
import pickle
import random
import time
import numpy as np
import torch
import torch.nn as nn
from simulator import PendulumSimulator, SimulatorConfig
from trainer_utils import create_fast_simulator, normalize_angle, load_training_params, send_training_update

try:
    from evotorch import Problem
    from evotorch.algorithms import SNES
    EVOTORCH_AVAILABLE = True
except ImportError:
    EVOTORCH_AVAILABLE = False
    print("Warning: EvoTorch not available. Install with: pip install evotorch")

# Paths
MODEL_PATH = os.path.join(os.path.dirname(__file__), 'evotorch_balance_model.pkl')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'evotorch_checkpoints')
GENERATION_HISTORY_FILE = os.path.join(os.path.dirname(__file__), 'evotorch_generation_history.json')
TRAINING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
STOP_TRAINING_FILE = os.path.join(os.path.dirname(__file__), 'stop_training.flag')

# Training parameters (defaults, can be overridden by config)
MAX_SPEED = 9000
SIMULATION_STEPS = 5000
GENERATIONS = 0  # 0 means indefinite training (run until stopped)
POPULATION_SIZE = 50
EVAL_DT = 0.02  # Evaluation timestep (50Hz)

# Network architecture
INPUT_SIZE = 5
HIDDEN_SIZE = 10
OUTPUT_SIZE = 1
NUM_HIDDEN_LAYERS = 2

# Progressive training parameters
PROGRESSIVE_TRAINING_GENERATIONS = 100  # Generations to gradually increase gravity
INITIAL_GRAVITY = 2.0  # Start with lower gravity (m/s²)
FINAL_GRAVITY = 9.81  # Final gravity (m/s²)
INITIAL_DAMPING = 0.5  # Start with higher damping
FINAL_DAMPING = 0.1  # Final damping (matches create_fast_simulator)


class BalancePolicy(nn.Module):
    """Neural network policy for balance control (2 hidden layers, 10 nodes each)"""
    
    def __init__(self, input_size=INPUT_SIZE, hidden_size=HIDDEN_SIZE, output_size=OUTPUT_SIZE):
        super(BalancePolicy, self).__init__()
        self.fc1 = nn.Linear(input_size, hidden_size)
        self.fc2 = nn.Linear(hidden_size, hidden_size)
        self.fc3 = nn.Linear(hidden_size, output_size)
        self.activation = nn.Tanh()
    
    def forward(self, x):
        x = self.activation(self.fc1(x))
        x = self.activation(self.fc2(x))
        x = torch.tanh(self.fc3(x))  # Output in [-1, 1] range
        return x


def get_progressive_physics(generation_num):
    """
    Calculate progressive gravity and damping values based on generation number.
    
    For generations 0 to PROGRESSIVE_TRAINING_GENERATIONS:
    - Gravity: linearly increases from INITIAL_GRAVITY to FINAL_GRAVITY
    - Damping: linearly decreases from INITIAL_DAMPING to FINAL_DAMPING
    
    After PROGRESSIVE_TRAINING_GENERATIONS, uses final values.
    
    Args:
        generation_num: Current generation number (0-indexed)
    
    Returns:
        Tuple of (gravity, damping)
    """
    if generation_num >= PROGRESSIVE_TRAINING_GENERATIONS:
        return FINAL_GRAVITY, FINAL_DAMPING
    
    # Linear interpolation
    progress = generation_num / PROGRESSIVE_TRAINING_GENERATIONS
    
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


def evaluate_policy_with_recording(policy_params, generation_num=0):
    """
    Evaluate a policy by running an episode and returning total reward and trajectory.
    
    Args:
        policy_params: Flattened policy parameters (1D array or numpy array)
        generation_num: Current generation number (for progressive physics)
    
    Returns:
        (total_reward, trajectory) where trajectory is a list of state dicts
    """
    trajectory = []
    
    try:
        # Convert to numpy array if needed
        if isinstance(policy_params, torch.Tensor):
            policy_params = policy_params.cpu().numpy()
        elif not isinstance(policy_params, np.ndarray):
            policy_params = np.array(policy_params)
        
        # Get progressive physics values
        gravity, damping = get_progressive_physics(generation_num)
        
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
        
        # Create policy network
        policy = BalancePolicy()
        
        # Set policy parameters
        param_idx = 0
        with torch.no_grad():
            for param in policy.parameters():
                param_size = param.numel()
                if param_idx + param_size > len(policy_params):
                    raise ValueError(f"Not enough parameters: need {param_idx + param_size}, got {len(policy_params)}")
                param.data = torch.tensor(
                    policy_params[param_idx:param_idx + param_size],
                    dtype=torch.float32
                ).reshape(param.shape)
                param_idx += param_size
        
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
            angle_rad = state['pendulum_angle']  # Already in radians
            
            # Record state for trajectory
            trajectory.append({
                'step': step,
                'angle': math.degrees(angle_rad),
                'angular_velocity': state['pendulum_velocity'],
                'cart_position': state['cart_position'],
                'cart_velocity': state['cart_velocity'],
                'time': step * EVAL_DT
            })
            
            # Prepare state vector using sin(θ) and cos(θ) to prevent angle wrap discontinuity
            state_vec = torch.tensor([
                math.sin(angle_rad),
                math.cos(angle_rad),
                state['pendulum_velocity'] / 1000.0,
                state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
                state['cart_velocity'] / MAX_SPEED
            ], dtype=torch.float32)
            
            # Get action from policy
            with torch.no_grad():
                action = policy(state_vec).item()
            
            # Convert action to acceleration
            max_accel = sim_config.motor_accel
            acceleration = action * max_accel
            
            # Record action
            trajectory[-1]['action'] = action
            trajectory[-1]['acceleration'] = acceleration
            
            # Apply action
            sim.set_target_acceleration(acceleration)
            sim.step(EVAL_DT)
            
            # Get next state
            next_state = sim.get_state()
            next_angle_rad = next_state['pendulum_angle']
            
            # Compute reward
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
            
            # Record reward
            trajectory[-1]['reward'] = reward
            
            # Check if done: |θ - π| > 20° or |x| > rail_limit
            angle_from_up_rad = abs(next_angle_rad - math.pi)
            cart_pos_normalized = abs(next_state['cart_position'] / (rail_length / 2)) if rail_length > 0 else 0
            done = (next_state['limit_left'] or next_state['limit_right'] or 
                    angle_from_up_rad > math.radians(20) or
                    cart_pos_normalized > 1.0)
            
            if done:
                trajectory[-1]['done'] = True
                break
        
        sim.close()
        return total_reward, trajectory
    except Exception as e:
        # If anything goes wrong, return a very low fitness
        print(f"Error in evaluate_policy_with_recording: {e}", flush=True)
        import traceback
        traceback.print_exc()
        return -1000.0, []


def evaluate_policy(policy_params, generation_num=0):
    """
    Evaluate a policy by running an episode and returning total reward.
    Uses evaluate_policy_with_recording but discards trajectory for backward compatibility.
    
    Args:
        policy_params: Flattened policy parameters (1D array or numpy array)
        generation_num: Current generation number (for progressive physics)
    
    Returns:
        Total reward for the episode
    """
    reward, _ = evaluate_policy_with_recording(policy_params, generation_num)
    return reward
    try:
        # Convert to numpy array if needed
        if isinstance(policy_params, torch.Tensor):
            policy_params = policy_params.cpu().numpy()
        elif not isinstance(policy_params, np.ndarray):
            policy_params = np.array(policy_params)
        
        # Get progressive physics values
        gravity, damping = get_progressive_physics(generation_num)
        
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
        
        # Create policy network
        policy = BalancePolicy()
        
        # Set policy parameters
        param_idx = 0
        with torch.no_grad():
            for param in policy.parameters():
                param_size = param.numel()
                if param_idx + param_size > len(policy_params):
                    raise ValueError(f"Not enough parameters: need {param_idx + param_size}, got {len(policy_params)}")
                param.data = torch.tensor(
                    policy_params[param_idx:param_idx + param_size],
                    dtype=torch.float32
                ).reshape(param.shape)
                param_idx += param_size
        
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
            angle_rad = state['pendulum_angle']  # Already in radians
            
            # Prepare state vector using sin(θ) and cos(θ) to prevent angle wrap discontinuity
            state_vec = torch.tensor([
                math.sin(angle_rad),
                math.cos(angle_rad),
                state['pendulum_velocity'] / 1000.0,
                state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
                state['cart_velocity'] / MAX_SPEED
            ], dtype=torch.float32)
            
            # Get action from policy
            with torch.no_grad():
                action = policy(state_vec).item()
            
            # Convert action to acceleration
            max_accel = sim_config.motor_accel
            acceleration = action * max_accel
            
            # Apply action
            sim.set_target_acceleration(acceleration)
            sim.step(EVAL_DT)
            
            # Get next state
            next_state = sim.get_state()
            next_angle_rad = next_state['pendulum_angle']
            
            # Compute reward
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
            
            if done:
                break
        
        sim.close()
        return total_reward
    except Exception as e:
        # If anything goes wrong, return a very low fitness
        print(f"Error in evaluate_policy: {e}", flush=True)
        import traceback
        traceback.print_exc()
        return -1000.0


def load_evotorch_config():
    """Load evotorch training config from JSON file"""
    global MAX_SPEED, SIMULATION_STEPS, GENERATIONS, POPULATION_SIZE
    
    try:
        if os.path.exists(TRAINING_CONFIG_FILE):
            with open(TRAINING_CONFIG_FILE, 'r') as f:
                config = json.load(f)
                evotorch_config = config.get('evotorch', {})
                MAX_SPEED = evotorch_config.get('max_speed', MAX_SPEED)
                SIMULATION_STEPS = evotorch_config.get('sim_steps', SIMULATION_STEPS)
                GENERATIONS = evotorch_config.get('generations', GENERATIONS)
                POPULATION_SIZE = evotorch_config.get('population_size', POPULATION_SIZE)
    except Exception as e:
        print(f"Warning: Could not load evotorch config: {e}")


def train():
    """Train the policy using EvoTorch"""
    global MAX_SPEED, SIMULATION_STEPS, GENERATIONS, POPULATION_SIZE
    
    if not EVOTORCH_AVAILABLE:
        print("Error: EvoTorch not available. Install with: pip install evotorch")
        return
    
    # Load config
    load_evotorch_config()
    
    # Clear stop flag
    if os.path.exists(STOP_TRAINING_FILE):
        os.remove(STOP_TRAINING_FILE)
    
    # Create policy to get parameter count
    policy = BalancePolicy()
    solution_length = sum(p.numel() for p in policy.parameters())
    
    print("Starting EvoTorch training...")
    print(f"Network architecture: {INPUT_SIZE} inputs -> {HIDDEN_SIZE} (hidden) -> {HIDDEN_SIZE} (hidden) -> {OUTPUT_SIZE} output")
    print(f"Total parameters: {solution_length}")
    if GENERATIONS > 0:
        print(f"Generations: {GENERATIONS}, Population: {POPULATION_SIZE}, Max Speed: {MAX_SPEED}, Sim Steps: {SIMULATION_STEPS}")
    else:
        print(f"Training: INDEFINITE (until stopped), Population: {POPULATION_SIZE}, Max Speed: {MAX_SPEED}, Sim Steps: {SIMULATION_STEPS}")
    
    # Create problem with custom evaluation
    class BalanceProblem(Problem):
        def __init__(self, generation_num=0):
            super().__init__(
                objective_sense='max',
                solution_length=solution_length,
                initial_bounds=(-1.0, 1.0),
                num_actors=0  # No parallelization
            )
            self.generation_num = generation_num
        
        def _evaluate(self, solution):
            """Evaluate a single solution"""
            try:
                # Extract solution values
                if hasattr(solution, 'values'):
                    params = solution.values.cpu().numpy()
                elif isinstance(solution, torch.Tensor):
                    params = solution.cpu().numpy()
                else:
                    params = np.array(solution)
                
                # Evaluate the policy
                reward = evaluate_policy(params, self.generation_num)
                
                # Ensure reward is a valid finite float
                if reward is None:
                    reward = -1000.0
                elif not isinstance(reward, (int, float)):
                    reward = float(reward)
                elif not np.isfinite(reward):
                    reward = -1000.0
                
                return float(reward)
            except Exception as e:
                print(f"Warning: Evaluation failed: {e}", flush=True)
                return -1000.0
        
        def _evaluate_batch(self, solutions):
            """Evaluate a batch of solutions and set evals on the SolutionBatch"""
            # solutions is a SolutionBatch object
            try:
                # Get the solution values as a batch tensor
                # solutions.values is shape (batch_size, solution_length)
                batch_values = solutions.values.cpu().numpy()
                batch_size = batch_values.shape[0]
                results = []
                
                # Record evaluation data for replay
                generation_records_dir = os.path.join(CHECKPOINT_DIR, f'generation_{self.generation_num + 1}_records')
                os.makedirs(generation_records_dir, exist_ok=True)
                
                for i in range(batch_size):
                    try:
                        # Get parameters for this solution
                        params = batch_values[i]
                        
                        # Evaluate the policy and record the trajectory
                        reward, trajectory = evaluate_policy_with_recording(params, self.generation_num)
                        
                        # Ensure reward is a valid finite float
                        if reward is None:
                            reward = -1000.0
                        elif not isinstance(reward, (int, float)):
                            reward = float(reward)
                        elif not np.isfinite(reward):
                            reward = -1000.0
                        
                        results.append(float(reward))
                        
                        # Save trajectory for replay
                        if trajectory:
                            record_file = os.path.join(generation_records_dir, f'solution_{i}.json')
                            with open(record_file, 'w') as f:
                                json.dump({
                                    'solution_id': i,
                                    'fitness': reward,
                                    'trajectory': trajectory,
                                    'generation': self.generation_num + 1
                                }, f, indent=2)
                    except Exception as e:
                        # If evaluation fails, give a very low fitness
                        print(f"Warning: Evaluation failed for solution {i}: {e}", flush=True)
                        results.append(-1000.0)
                
                # Convert to tensor with shape (batch_size, 1) for single-objective problems
                evals = torch.tensor(results, dtype=torch.float32, device=self.device)
                if evals.ndim == 1:
                    evals = evals.unsqueeze(1)  # Convert to (n, 1) shape
                
                # Set the evaluation results on the SolutionBatch
                solutions.set_evals(evals)
                
            except Exception as e:
                print(f"ERROR in _evaluate_batch: {e}", flush=True)
                import traceback
                traceback.print_exc()
                # Set low fitness values on the batch
                batch_size = len(solutions) if hasattr(solutions, '__len__') else 1
                evals = torch.full((batch_size, 1), -1000.0, dtype=torch.float32, device=self.device)
                solutions.set_evals(evals)
    
    # Create problem
    problem = BalanceProblem(generation_num=0)
    
    # Create searcher (using SNES - Separable Natural Evolution Strategy)
    # stdev_init controls the initial search radius (standard deviation)
    searcher = SNES(problem, popsize=POPULATION_SIZE, stdev_init=0.5)
    
    best_fitness = float('-inf')
    generation = 0
    
    # Train indefinitely (until stopped via stop flag)
    while True:
        # Check for stop flag
        if os.path.exists(STOP_TRAINING_FILE):
            print("Stop flag detected, stopping training...")
            break
        
        # If GENERATIONS is set and we've reached it, stop
        if GENERATIONS > 0 and generation >= GENERATIONS:
            print(f"Reached {GENERATIONS} generations, stopping training...")
            break
        
        # Update generation number in problem
        problem.generation_num = generation
        
        # Run one generation
        searcher.run(1)
        
        # Get best solution
        best_solution = searcher.status['best']
        # Re-evaluate to get current fitness
        try:
            if hasattr(best_solution, 'values'):
                best_params = best_solution.values.cpu().numpy()
            elif isinstance(best_solution, torch.Tensor):
                best_params = best_solution.cpu().numpy()
            else:
                best_params = np.array(best_solution)
            current_fitness = evaluate_policy(best_params, generation)
        except Exception as e:
            print(f"Warning: Failed to evaluate best solution: {e}", flush=True)
            current_fitness = float('-inf')
        
        # Update best fitness tracking
        if current_fitness > best_fitness:
            best_fitness = current_fitness
            print(f"Generation {generation + 1}, New best fitness: {best_fitness:.1f}")
        
        # Save latest model (not just best)
        policy = BalancePolicy()
        param_idx = 0
        try:
            # Get current best solution parameters
            if hasattr(best_solution, 'values'):
                best_params = best_solution.values.cpu().numpy()
            elif isinstance(best_solution, torch.Tensor):
                best_params = best_solution.cpu().numpy()
            else:
                best_params = np.array(best_solution)
            
            with torch.no_grad():
                for param in policy.parameters():
                    param_size = param.numel()
                    param.data = torch.tensor(
                        best_params[param_idx:param_idx + param_size],
                        dtype=torch.float32
                    ).reshape(param.shape)
                    param_idx += param_size
        except Exception as e:
            print(f"Warning: Failed to save model: {e}", flush=True)
        else:
            # Save the latest model
            with open(MODEL_PATH, 'wb') as f:
                pickle.dump(policy, f)
            
            # Save checkpoint for this generation
            gen_num = generation + 1
            os.makedirs(CHECKPOINT_DIR, exist_ok=True)
            checkpoint_path = os.path.join(CHECKPOINT_DIR, f'generation_{gen_num}.pkl')
            with open(checkpoint_path, 'wb') as f:
                pickle.dump(policy, f)
            
            # Update generation history
            history = {}
            if os.path.exists(GENERATION_HISTORY_FILE):
                try:
                    with open(GENERATION_HISTORY_FILE, 'r') as f:
                        history = json.load(f)
                except:
                    history = {}
            
            history[str(gen_num)] = {
                'generation': gen_num,
                'fitness': current_fitness,
                'best_fitness': best_fitness,
                'checkpoint': checkpoint_path,
                'timestamp': time.time()
            }
            
            with open(GENERATION_HISTORY_FILE, 'w') as f:
                json.dump(history, f, indent=2)
            
            if (generation + 1) % 10 == 0:
                print(f"Generation {generation + 1}, Saved latest model (fitness: {current_fitness:.1f})")
        
        # Send training update
        send_training_update({
            'trainer': 'evotorch',
            'generation': generation + 1,
            'best_fitness': best_fitness,
            'current_fitness': current_fitness,
            'population_size': POPULATION_SIZE
        })
        
        if (generation + 1) % 10 == 0:
            # Get current physics values for display
            current_gravity, current_damping = get_progressive_physics(generation)
            print(f"Generation {generation + 1}, Best: {best_fitness:.1f}, Current: {current_fitness:.1f}, "
                  f"Gravity: {current_gravity:.2f}, Damping: {current_damping:.3f}")
        
        generation += 1
    
    print(f"Training stopped. Best fitness: {best_fitness:.1f}")
    if os.path.exists(MODEL_PATH):
        print(f"Best model saved to {MODEL_PATH}")


def test():
    """Test the trained model"""
    if not os.path.exists(MODEL_PATH):
        print("No trained model found. Run training first.")
        return
    
    # Load model
    with open(MODEL_PATH, 'rb') as f:
        policy = pickle.load(f)
    
    policy.eval()
    
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
        state_vec = torch.tensor([
            math.sin(angle_rad),
            math.cos(angle_rad),
            state['pendulum_velocity'] / 1000.0,
            state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
            state['cart_velocity'] / MAX_SPEED
        ], dtype=torch.float32)
        
        # Get action from policy
        with torch.no_grad():
            action = policy(state_vec).item()
        
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

