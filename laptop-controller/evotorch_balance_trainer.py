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
from trainer_utils import send_training_update

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

# Progressive training parameters
PROGRESSIVE_TRAINING_GENERATIONS = 100
INITIAL_GRAVITY = 2.0
FINAL_GRAVITY = 9.81
INITIAL_DAMPING = 0.5
FINAL_DAMPING = 0.1


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
    """Calculate progressive gravity and damping values based on generation number"""
    if generation_num >= PROGRESSIVE_TRAINING_GENERATIONS:
        return FINAL_GRAVITY, FINAL_DAMPING
    
    progress = generation_num / PROGRESSIVE_TRAINING_GENERATIONS
    gravity = INITIAL_GRAVITY + (FINAL_GRAVITY - INITIAL_GRAVITY) * progress
    damping = INITIAL_DAMPING + (FINAL_DAMPING - INITIAL_DAMPING) * progress
    return gravity, damping


def compute_reward(state, angle_rad, angular_velocity, cart_position, cart_velocity, acceleration, rail_length, max_accel):
    """Compute reward for balancing task"""
    # Reward coefficients
    k1, k2, k3, k4 = 0.01, 0.001, 0.01, 0.001
    
    # Upright term: -cos(θ) (max when θ=π, upright)
    upright_term = -math.cos(angle_rad)
    
    # Normalize and compute penalties
    angular_vel_sq = angular_velocity ** 2
    cart_pos_normalized = cart_position / (rail_length / 2) if rail_length > 0 else 0
    cart_pos_sq = cart_pos_normalized ** 2
    cart_vel_normalized = cart_velocity / MAX_SPEED
    cart_vel_sq = cart_vel_normalized ** 2
    accel_normalized = acceleration / max_accel
    accel_sq = accel_normalized ** 2
    
    # Base reward
    reward = upright_term - k1 * angular_vel_sq - k2 * cart_pos_sq - k3 * cart_vel_sq - k4 * accel_sq
    
    # Terminal penalties
    angle_from_up_rad = abs(angle_rad - math.pi)
    if angle_from_up_rad > math.radians(20) or state['limit_left'] or state['limit_right']:
        reward = -50.0
    
    return reward


def extract_params(solution):
    """Extract parameters from solution object"""
    if hasattr(solution, 'values'):
        return solution.values.cpu().numpy()
    elif isinstance(solution, torch.Tensor):
        return solution.cpu().numpy()
    else:
        return np.array(solution)


def set_policy_params(policy, params):
    """Set policy parameters from flattened array"""
    param_idx = 0
    with torch.no_grad():
        for param in policy.parameters():
            param_size = param.numel()
            if param_idx + param_size > len(params):
                raise ValueError(f"Not enough parameters: need {param_idx + param_size}, got {len(params)}")
            param.data = torch.tensor(params[param_idx:param_idx + param_size], dtype=torch.float32).reshape(param.shape)
            param_idx += param_size


def create_simulator(generation_num):
    """Create simulator with progressive physics"""
    gravity, damping = get_progressive_physics(generation_num)
    sim_config = SimulatorConfig(
        rail_length_steps=7500,
        cart_mass=0.5,
        motor_accel=500000,
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=gravity,
        damping=damping,
    )
    return PendulumSimulator(sim_config, start_background_thread=False), sim_config


def prepare_state_vector(state, rail_length):
    """Prepare normalized state vector for policy input"""
    angle_rad = state['pendulum_angle']
    return torch.tensor([
        math.sin(angle_rad),
        math.cos(angle_rad),
        state['pendulum_velocity'] / 1000.0,
        state['cart_position'] / (rail_length / 2) if rail_length > 0 else 0,
        state['cart_velocity'] / MAX_SPEED
    ], dtype=torch.float32)


def check_done(state, rail_length):
    """Check if episode should terminate"""
    angle_from_up_rad = abs(state['pendulum_angle'] - math.pi)
    cart_pos_normalized = abs(state['cart_position'] / (rail_length / 2)) if rail_length > 0 else 0
    return (state['limit_left'] or state['limit_right'] or 
            angle_from_up_rad > math.radians(20) or
            cart_pos_normalized > 1.0)


def evaluate_policy(policy_params, generation_num=0, record_trajectory=False):
    """
    Evaluate a policy by running an episode.
    
    Args:
        policy_params: Flattened policy parameters (1D array or numpy array)
        generation_num: Current generation number (for progressive physics)
        record_trajectory: If True, return trajectory data
    
    Returns:
        Total reward (and trajectory if record_trajectory=True)
    """
    trajectory = [] if record_trajectory else None
    
    try:
        # Convert to numpy array
        if isinstance(policy_params, torch.Tensor):
            policy_params = policy_params.cpu().numpy()
        elif not isinstance(policy_params, np.ndarray):
            policy_params = np.array(policy_params)
        
        # Create simulator
        sim, sim_config = create_simulator(generation_num)
        
        # Create and configure policy
        policy = BalancePolicy()
        set_policy_params(policy, policy_params)
        
        # Initialize state
        perturbation = random.uniform(-10, 10)
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
        max_accel = sim_config.motor_accel
        total_reward = 0.0
        
        # Run simulation
        for step in range(SIMULATION_STEPS):
            state = sim.get_state()
            angle_rad = state['pendulum_angle']
            
            # Record state if needed
            if record_trajectory:
                trajectory.append({
                    'step': step,
                    'angle': math.degrees(angle_rad),
                    'angular_velocity': state['pendulum_velocity'],
                    'cart_position': state['cart_position'],
                    'cart_velocity': state['cart_velocity'],
                    'time': step * EVAL_DT
                })
            
            # Get action from policy
            state_vec = prepare_state_vector(state, rail_length)
            with torch.no_grad():
                action = policy(state_vec).item()
            
            acceleration = action * max_accel
            
            # Record action if needed
            if record_trajectory:
                trajectory[-1]['action'] = action
                trajectory[-1]['acceleration'] = acceleration
            
            # Apply action
            sim.set_target_acceleration(acceleration)
            sim.step(EVAL_DT)
            
            # Get next state and compute reward
            next_state = sim.get_state()
            reward = compute_reward(
                next_state, next_state['pendulum_angle'],
                next_state['pendulum_velocity'], next_state['cart_position'],
                next_state['cart_velocity'], acceleration, rail_length, max_accel
            )
            total_reward += reward
            
            if record_trajectory:
                trajectory[-1]['reward'] = reward
            
            # Check termination
            if check_done(next_state, rail_length):
                if record_trajectory:
                    trajectory[-1]['done'] = True
                break
        
        sim.close()
        return (total_reward, trajectory) if record_trajectory else total_reward
        
    except Exception as e:
        print(f"Error in evaluate_policy: {e}", flush=True)
        import traceback
        traceback.print_exc()
        return (-1000.0, []) if record_trajectory else -1000.0


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


def validate_reward(reward):
    """Validate and normalize reward value"""
    if reward is None or not isinstance(reward, (int, float)) or not np.isfinite(reward):
        return -1000.0
    return float(reward)


def train():
    """Train the policy using EvoTorch"""
    global MAX_SPEED, SIMULATION_STEPS, GENERATIONS, POPULATION_SIZE
    
    if not EVOTORCH_AVAILABLE:
        print("Error: EvoTorch not available. Install with: pip install evotorch")
        return
    
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
            import multiprocessing
            num_actors = min(multiprocessing.cpu_count(), POPULATION_SIZE)
            super().__init__(
                objective_sense='max',
                solution_length=solution_length,
                initial_bounds=(-1.0, 1.0),
                num_actors=num_actors
            )
            self.generation_num = generation_num
            print(f"Using {num_actors} parallel workers for evaluation")
        
        def _evaluate(self, solution):
            """Evaluate a single solution"""
            try:
                params = extract_params(solution)
                reward = evaluate_policy(params, self.generation_num)
                return validate_reward(reward)
            except Exception as e:
                print(f"Warning: Evaluation failed: {e}", flush=True)
                return -1000.0
        
        def _evaluate_batch(self, solutions):
            """Evaluate a batch of solutions and set evals on the SolutionBatch"""
            try:
                batch_values = solutions.values.cpu().numpy()
                batch_size = batch_values.shape[0]
                results = []
                record_top_n = max(5, batch_size // 10)
                
                # Create records directory
                generation_records_dir = os.path.join(CHECKPOINT_DIR, f'generation_{self.generation_num + 1}_records')
                os.makedirs(generation_records_dir, exist_ok=True)
                
                # First pass: evaluate all solutions
                for i in range(batch_size):
                    try:
                        reward = evaluate_policy(batch_values[i], self.generation_num)
                        results.append(validate_reward(reward))
                    except Exception as e:
                        print(f"Warning: Evaluation failed for solution {i}: {e}", flush=True)
                        results.append(-1000.0)
                
                # Second pass: record trajectories for top solutions
                if batch_size > 0:
                    sorted_indices = sorted(range(batch_size), key=lambda i: results[i], reverse=True)
                    for rank, idx in enumerate(sorted_indices[:record_top_n]):
                        try:
                            reward, trajectory = evaluate_policy(batch_values[idx], self.generation_num, record_trajectory=True)
                            if trajectory:
                                record_file = os.path.join(generation_records_dir, f'solution_{idx}.json')
                                with open(record_file, 'w') as f:
                                    json.dump({
                                        'solution_id': idx,
                                        'fitness': reward,
                                        'trajectory': trajectory,
                                        'generation': self.generation_num + 1,
                                        'rank': rank + 1
                                    }, f, indent=2)
                        except Exception as e:
                            print(f"Warning: Failed to record trajectory for solution {idx}: {e}", flush=True)
                
                # Set evaluation results
                evals = torch.tensor(results, dtype=torch.float32, device=self.device)
                if evals.ndim == 1:
                    evals = evals.unsqueeze(1)
                solutions.set_evals(evals)
                
            except Exception as e:
                print(f"ERROR in _evaluate_batch: {e}", flush=True)
                import traceback
                traceback.print_exc()
                batch_size = len(solutions) if hasattr(solutions, '__len__') else 1
                evals = torch.full((batch_size, 1), -1000.0, dtype=torch.float32, device=self.device)
                solutions.set_evals(evals)
    
    # Create problem and searcher
    problem = BalanceProblem(generation_num=0)
    searcher = SNES(problem, popsize=POPULATION_SIZE, stdev_init=0.5)
    
    best_fitness = float('-inf')
    generation = 0
    
    # Training loop
    while True:
        if os.path.exists(STOP_TRAINING_FILE):
            print("Stop flag detected, stopping training...")
            break
        
        if GENERATIONS > 0 and generation >= GENERATIONS:
            print(f"Reached {GENERATIONS} generations, stopping training...")
            break
        
        problem.generation_num = generation
        searcher.run(1)
        
        # Get best solution and evaluate
        best_solution = searcher.status['best']
        try:
            best_params = extract_params(best_solution)
            current_fitness = evaluate_policy(best_params, generation)
            current_fitness = validate_reward(current_fitness)
        except Exception as e:
            print(f"Warning: Failed to evaluate best solution: {e}", flush=True)
            current_fitness = float('-inf')
        
        # Update best fitness
        if current_fitness > best_fitness:
            best_fitness = current_fitness
            print(f"Generation {generation + 1}, New best fitness: {best_fitness:.1f}")
        
        # Save model
        try:
            policy = BalancePolicy()
            set_policy_params(policy, best_params)
            
            with open(MODEL_PATH, 'wb') as f:
                pickle.dump(policy, f)
            
            # Save checkpoint
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
                    pass
            
            history[str(gen_num)] = {
                'generation': gen_num,
                'fitness': current_fitness,
                'best_fitness': best_fitness,
                'checkpoint': checkpoint_path,
                'timestamp': time.time()
            }
            
            with open(GENERATION_HISTORY_FILE, 'w') as f:
                json.dump(history, f, indent=2)
            
            if gen_num % 10 == 0:
                print(f"Generation {gen_num}, Saved latest model (fitness: {current_fitness:.1f})")
        except Exception as e:
            print(f"Warning: Failed to save model: {e}", flush=True)
        
        # Send training update
        send_training_update({
            'trainer': 'evotorch',
            'generation': generation + 1,
            'best_fitness': best_fitness,
            'current_fitness': current_fitness,
            'population_size': POPULATION_SIZE
        })
        
        if (generation + 1) % 10 == 0:
            gravity, damping = get_progressive_physics(generation)
            print(f"Generation {generation + 1}, Best: {best_fitness:.1f}, Current: {current_fitness:.1f}, "
                  f"Gravity: {gravity:.2f}, Damping: {damping:.3f}")
        
        generation += 1
    
    print(f"Training stopped. Best fitness: {best_fitness:.1f}")
    if os.path.exists(MODEL_PATH):
        print(f"Best model saved to {MODEL_PATH}")


def test():
    """Test the trained model"""
    if not os.path.exists(MODEL_PATH):
        print("No trained model found. Run training first.")
        return
    
    with open(MODEL_PATH, 'rb') as f:
        policy = pickle.load(f)
    
    policy.eval()
    
    sim_config = SimulatorConfig(
        rail_length_steps=7500,
        cart_mass=0.5,
        motor_accel=500000,
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=9.81,
        damping=0.1,
    )
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
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
        
        state_vec = prepare_state_vector(state, rail_length)
        
        with torch.no_grad():
            action = policy(state_vec).item()
        
        acceleration = action * sim_config.motor_accel
        
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
