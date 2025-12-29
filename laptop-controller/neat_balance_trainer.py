"""
NEAT-Python Trainer for Inverted Pendulum BALANCE ONLY

Trains a neural network to keep the pendulum balanced when starting
from the upright position (180°). Simpler task than swing-up.

Inputs (4):
  - Angle from upright (normalized: 180° = 0, 0°/360° = ±1)
  - Angular velocity (normalized)
  - Cart position (normalized)
  - Cart velocity (normalized)

Output (1):
  - Cart velocity command (-1 to 1, scaled to max speed)

Usage:
  python neat_balance_trainer.py              # Train new network
  python neat_balance_trainer.py --continue   # Continue from checkpoint
  python neat_balance_trainer.py --test       # Test best genome
"""

import os
import sys
import math
import pickle
import random
import neat
import time
import json
from simulator import PendulumSimulator
from trainer_utils import (
    normalize_angle,
    create_fast_simulator,
    load_training_params,
    send_training_update,
    eval_genomes as eval_genomes_base
)

# Paths
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'neat_balance_config.txt')
BEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), 'best_balance_genome.pkl')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'neat_balance_checkpoints')

# Training stats
training_stats = {
    "generation": 0,
    "best_fitness": 0,
    "avg_fitness": 0,
    "species_count": 0,
    "population_size": 0,
    "running": True
}

# Load training parameters from JSON config
_params = load_training_params("balance")
MAX_SPEED = _params['max_speed']
SIMULATION_STEPS = _params['sim_steps']
EVAL_DT = 0.02  # Evaluation timestep (50Hz)

# Perturbation parameters (reloadable during training)
_last_loaded_pert = None

def get_perturbation_params():
    """Get current perturbation parameters from config (allows real-time updates)"""
    global _last_loaded_pert
    params = load_training_params("balance")
    angle_pert = params.get('angle_perturbation', 10)
    vel_pert = params.get('velocity_perturbation', 0.5)
    # Print when values change or on first load
    if _last_loaded_pert is None or _last_loaded_pert != (angle_pert, vel_pert):
        print(f"[Perturbation] Loaded: angle={angle_pert}°, vel={vel_pert} rad/s", flush=True)
        _last_loaded_pert = (angle_pert, vel_pert)
    
    return {
        'angle_perturbation': angle_pert,
        'velocity_perturbation': vel_pert
    }

def evaluate_genome(genome, config):
    """
    Evaluate a single genome for BALANCE task.
    Starts pendulum near 180° (upright) with small perturbation.
    
    Fitness = total time steps the pendulum stays balanced
    """
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    
    # Create simulator (no background thread for training)
    sim_config = create_fast_simulator()
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Get current perturbation parameters (allows real-time adjustment)
    pert_params = get_perturbation_params()
    angle_pert = pert_params['angle_perturbation']
    vel_pert = pert_params['velocity_perturbation']
    
    # Start pendulum at 180° (upright) with configurable random perturbation
    angle_perturbation = random.uniform(-angle_pert, angle_pert)
    velocity_perturbation = random.uniform(-vel_pert, vel_pert)
    
    sim.set_state(
        cart_position=0.0,
        cart_velocity=0.0,
        pendulum_angle=math.radians(180 + angle_perturbation),
        pendulum_velocity=velocity_perturbation
    )
    
    # Get rail limits
    state = sim.get_state()
    if not isinstance(state, dict) or 'limit_right_pos' not in state or 'limit_left_pos' not in state:
        # Fallback: use config values
        rail_length = sim_config.rail_length_steps
    else:
        rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    fitness = 0.0
    
    for step in range(SIMULATION_STEPS):
        # Get current state from simulator
        state = sim.get_state()
        if not isinstance(state, dict):
            print(f"ERROR: get_state() returned non-dict: {type(state)} = {state}", flush=True)
            raise ValueError(f"Invalid state from simulator: {state}")
        if 'pendulum_angle' not in state:
            print(f"ERROR: get_state() missing 'pendulum_angle'. Keys: {list(state.keys())}", flush=True)
            raise ValueError(f"Invalid state from simulator - missing pendulum_angle: {state}")
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        
        # Prepare inputs for neural network
        inputs = [
            normalize_angle(angle_deg),                          # Angle from upright (-1 to 1)
            state['pendulum_velocity'] / 1000.0,                    # Angular velocity (normalized)
            state['cart_position'] / (rail_length / 2),          # Position (normalized)
            state['cart_velocity'] / MAX_SPEED,                   # Cart velocity (normalized)
        ]
        
        # Get network output
        output = net.activate(inputs)
        target_velocity = output[0] * MAX_SPEED
        target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
        
        # Set target velocity and step physics
        sim.set_target_velocity(target_velocity)
        sim.step(EVAL_DT)
        
        # --- Fitness calculation ---
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        angle_from_up = abs(normalize_angle(angle_deg))  # 0 = at 180°, 1 = at 0°
        
        # Simple fitness: +1 for each step within 30° of upright
        if angle_from_up < 0.167:  # 30°/180° ≈ 0.167
            fitness += 1.0
        
        # Bonus for being near center position (encourages staying in middle of rail)
        # cart_position = state['cart_position']
        # # Normalize position: 0 = center, ±1 = at limits
        # normalized_pos = abs(cart_position / (rail_length / 2)) if rail_length > 0 else 0
        # # Bonus: max 0.005 points when at center, decreases as you move away
        # center_bonus = 0.05 * (1.0 - min(1.0, normalized_pos))
        # fitness += center_bonus
        
        # Stop early if fell too far (past 45°)
        if angle_from_up > 0.25:  # 45°/180° ≈ 0.25
            fitness -= 100  # Penalty for falling too far
            break
        
        # Stop if hit limit
        if state['limit_left'] or state['limit_right']:
            fitness = -100  # Penalty for hitting limit
            break
    
    sim.close()
    
    # Fitness bonus for higher perturbations (encourages training with more challenging conditions)
    # Use absolute values directly (not normalized)
    # Angle perturbation: in degrees, Velocity perturbation: in rad/s
    angle_pert_bonus = abs(angle_perturbation)
    vel_pert_bonus = abs(velocity_perturbation) * 10
    
    # Bonus: angle perturbation in degrees + velocity perturbation in rad/s
    perturbation_bonus = angle_pert_bonus + vel_pert_bonus
    fitness += perturbation_bonus
    
    return fitness


def eval_genomes(genomes, config):
    """Evaluate all genomes in the population"""
    eval_genomes_base(genomes, config, evaluate_genome)


class BalanceReporter(neat.reporting.BaseReporter):
    """Reporter that sends training stats and auto-saves best genome"""
    
    def __init__(self):
        self.generation = 0
        self.best_fitness_ever = float('-inf')
    
    def start_generation(self, generation):
        self.generation = generation
    
    def post_evaluate(self, config, population, species_set, best_genome):
        """Called after each generation's evaluation"""
        global training_stats
        
        # Calculate stats
        fitnesses = [g.fitness for g in population.values() if g.fitness is not None]
        current_best = best_genome.fitness if best_genome.fitness else 0
        
        training_stats["generation"] = self.generation
        training_stats["best_fitness"] = current_best
        training_stats["avg_fitness"] = sum(fitnesses) / len(fitnesses) if fitnesses else 0
        training_stats["species_count"] = len(species_set.species)
        training_stats["population_size"] = len(population)
        
        # Clear best_genome - only set if new best found
        training_stats.pop("best_genome", None)
        
        # Auto-save if this is a new best fitness
        if current_best > self.best_fitness_ever:
            self.best_fitness_ever = current_best
            with open(BEST_GENOME_PATH, 'wb') as f:
                pickle.dump(best_genome, f)
            print(f"*** New best fitness: {current_best:.1f} - Auto-saved! ***")
            
            # Update best genome info for web interface
            import datetime
            num_nodes = len(best_genome.nodes)
            num_connections = len([c for c in best_genome.connections.values() if c.enabled])
            training_stats["best_genome"] = {
                "fitness": current_best,
                "nodes": num_nodes,
                "connections": num_connections,
                "saved_at": datetime.datetime.now().strftime("%Y-%m-%d %H:%M")
            }
        
        # Write status to file
        send_training_update(training_stats)


def run_training(continue_from_checkpoint=False):
    """Run the NEAT training for balance task"""
    print("=" * 60)
    print("NEAT Training for Inverted Pendulum BALANCE")
    print("(Starting from upright position)")
    print("=" * 60)
    
    # Load NEAT configuration
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        CONFIG_PATH
    )
    
    # Create population
    if continue_from_checkpoint and os.path.exists(CHECKPOINT_DIR):
        checkpoints = [f for f in os.listdir(CHECKPOINT_DIR) if f.startswith('neat-checkpoint-')]
        if checkpoints:
            latest = max(checkpoints, key=lambda x: int(x.split('-')[-1]))
            checkpoint_path = os.path.join(CHECKPOINT_DIR, latest)
            print(f"Continuing from checkpoint: {checkpoint_path}")
            pop = neat.Checkpointer.restore_checkpoint(checkpoint_path)
        else:
            print("No checkpoints found, starting fresh")
            pop = neat.Population(config)
    else:
        pop = neat.Population(config)
    
    # Add reporters
    pop.add_reporter(neat.StdOutReporter(True))
    stats = neat.StatisticsReporter()
    pop.add_reporter(stats)
    pop.add_reporter(BalanceReporter())
    
    # Create checkpoint directory
    os.makedirs(CHECKPOINT_DIR, exist_ok=True)
    checkpointer = neat.Checkpointer(
        generation_interval=10,
        filename_prefix=os.path.join(CHECKPOINT_DIR, 'neat-checkpoint-')
    )
    pop.add_reporter(checkpointer)
    
    # Run evolution
    print("\nStarting evolution...")
    print("Press Ctrl+C to stop and save best genome\n")
    
    try:
        winner = pop.run(eval_genomes, n=999999)
    except KeyboardInterrupt:
        print("\n\nTraining interrupted!")
        winner = stats.best_genome()
    
    # Save the best genome
    print(f"\nBest genome fitness: {winner.fitness}")
    with open(BEST_GENOME_PATH, 'wb') as f:
        pickle.dump(winner, f)
    print(f"Best genome saved to: {BEST_GENOME_PATH}")
    
    return winner


def test_best_genome():
    """Test the best genome with visualization output"""
    print("Testing best balance genome...")
    
    if not os.path.exists(BEST_GENOME_PATH):
        print(f"Error: No best genome found at {BEST_GENOME_PATH}")
        print("Run training first: python neat_balance_trainer.py")
        return
    
    # Load config and genome
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        CONFIG_PATH
    )
    
    with open(BEST_GENOME_PATH, 'rb') as f:
        genome = pickle.load(f)
    
    print(f"Loaded genome with fitness: {genome.fitness}")
    
    # Create network
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    
    # Create simulator
    sim_config = create_fast_simulator()
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Start pendulum at 180° (upright) with small perturbation
    sim.set_state(
        cart_position=0.0,
        cart_velocity=0.0,
        pendulum_angle=math.radians(180 + 5),  # Start 5° off vertical
        pendulum_velocity=0.0
    )
    
    # Get rail limits
    state = sim.get_state()
    if 'limit_right_pos' not in state or 'limit_left_pos' not in state:
        # Fallback: use config values
        rail_length = sim_config.rail_length_steps
    else:
        rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    print("\nRunning simulation (Ctrl+C to stop)...")
    print("Time(s) | Angle(°) | Velocity | Cart Pos")
    print("-" * 45)
    
    try:
        for step in range(SIMULATION_STEPS * 2):
            # Get current state from simulator
            state = sim.get_state()
            angle_deg = math.degrees(state['pendulum_angle']) % 360
            
            # Prepare inputs for neural network (4 inputs)
            inputs = [
                normalize_angle(angle_deg),
                state['pendulum_velocity'] / 1000.0,
                state['cart_position'] / (rail_length / 2),
                state['cart_velocity'] / MAX_SPEED,
            ]
            
            # Get network output
            output = net.activate(inputs)
            target_velocity = output[0] * MAX_SPEED
            target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
            
            # Set target velocity and step physics
            sim.set_target_velocity(target_velocity)
            sim.step(EVAL_DT)
            
            # Print every 0.5 seconds
            if step % 25 == 0:
                state = sim.get_state()
                time_s = step * EVAL_DT
                angle_display = math.degrees(state['pendulum_angle'])
                print(f"{time_s:7.1f} | {angle_display:8.1f} | {target_velocity:8.0f} | {state['cart_position']:8.0f}")
            
            time.sleep(EVAL_DT / 10)  # Speed up visualization
            
    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        sim.close()


if __name__ == "__main__":
    if len(sys.argv) > 1:
        if sys.argv[1] == "--continue":
            run_training(continue_from_checkpoint=True)
        elif sys.argv[1] == "--test":
            test_best_genome()
        else:
            print(f"Unknown argument: {sys.argv[1]}")
            print("Usage: python neat_balance_trainer.py [--continue|--test]")
    else:
        run_training(continue_from_checkpoint=False)

