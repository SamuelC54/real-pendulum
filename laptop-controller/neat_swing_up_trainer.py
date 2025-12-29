"""
NEAT-Python Trainer for Inverted Pendulum SWING-UP

Trains a neural network to swing the pendulum from bottom (0°) to top (180°)
using the least amount of cart velocity/energy possible.

Fitness rewards:
  - Reaching 180° (upright position)
  - Using minimal cart velocity during the swing-up
  - Reaching top quickly

Inputs (4):
  - Angle from upright (normalized: 180° = 0, 0°/360° = ±1)
  - Angular velocity (normalized)
  - Cart position (normalized)
  - Cart velocity (normalized)

Output (1):
  - Cart velocity command (-1 to 1, scaled to max speed)

Usage:
  python neat_swing_up_trainer.py              # Train new network
  python neat_swing_up_trainer.py --continue   # Continue from checkpoint
  python neat_swing_up_trainer.py --test       # Test best genome
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
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'neat_swing_up_config.txt')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'neat_swing_up_checkpoints')
BEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), 'best_swing_up_genome.pkl')

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
_params = load_training_params("swing_up")
MAX_SPEED = _params['max_speed']
SIMULATION_STEPS = _params['sim_steps']
EVAL_DT = 0.02  # Evaluation timestep (50Hz)


eval_count = 0  # Global counter for logging

def evaluate_genome(genome, config, visualize=False):
    """
    Evaluate a genome for SWING-UP task.
    
    Goal: Swing pendulum from 0° (down) to 180° (up) with minimal velocity usage.
    
    Fitness rewards:
    - Big bonus for reaching 180° (upright)
    - Bonus for reaching top quickly (time bonus)
    - Penalty for using high cart velocities
    """
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    sim_config = create_fast_simulator()
    
    # Create simulator instance (no background thread for training)
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Start pendulum at BOTTOM (0°) with small random offset
    sim.set_state(
        cart_position=0.0,
        cart_velocity=0.0,
        pendulum_angle=math.radians(random.uniform(-5, 5)),  # Near 0° (down)
        pendulum_velocity=0.0
    )
    
    # Get rail limits
    state = sim.get_state()
    if 'limit_right_pos' not in state or 'limit_left_pos' not in state:
        # Fallback: use config values
        rail_length = sim_config.rail_length_steps
    else:
        rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    # Tracking variables
    max_velocity_used = 0.0  # Track maximum velocity used
    reached_top = False
    step_reached_top = SIMULATION_STEPS  # When did it reach top
    
    for step in range(SIMULATION_STEPS):
        # Get current state from simulator
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        
        # Prepare inputs for neural network
        inputs = [
            normalize_angle(angle_deg),                          # Angle from upright (-1 to 1)
            state['pendulum_velocity'] / 10.0,                  # Angular velocity (normalized)
            state['cart_position'] / (rail_length / 2),          # Position (normalized)
            state['cart_velocity'] / MAX_SPEED,                  # Cart velocity (normalized)
        ]
        
        # Get network output
        output = net.activate(inputs)
        target_velocity = output[0] * MAX_SPEED
        target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
        
        # Track velocity usage
        if abs(target_velocity) > max_velocity_used:
            max_velocity_used = abs(target_velocity)
        
        # Set target velocity and step physics
        sim.set_target_velocity(target_velocity)
        sim.step(EVAL_DT)
        
        # Check if reached top (within 10° of 180°)
        state = sim.get_state()
        angle_deg = math.degrees(state['pendulum_angle']) % 360
        angle_from_up = abs(normalize_angle(angle_deg))
        
        if angle_from_up < 0.056 and not reached_top:  # Within ~10° of 180°
            reached_top = True
            step_reached_top = step
        
        # Stop early if hit limit
        if state['limit_left'] or state['limit_right']:
            break
    
    # Get final state for fitness calculation
    final_state = sim.get_state()
    sim.close()
    
    # --- Calculate fitness ---
    fitness = 0.0
    
    if reached_top:
        # Base reward for reaching top
        fitness += 1000.0
        
        # Time bonus: faster is better (max 500 bonus if reached in first 100 steps)
        time_bonus = max(0, 500 - step_reached_top * 0.5)
        fitness += time_bonus
        
        # Efficiency bonus: lower max velocity = more points
        # If max velocity was only 2000, get 300 bonus. If it was 9000 (max), get 0.
        efficiency_bonus = max(0, 300 * (1 - max_velocity_used / MAX_SPEED))
        fitness += efficiency_bonus
    else:
        # Didn't reach top - give partial credit based on highest angle reached
        final_angle_deg = math.degrees(final_state['pendulum_angle']) % 360
        angle_from_up = abs(normalize_angle(final_angle_deg))
        closeness = 1.0 - angle_from_up  # 0 at bottom, 1 at top
        fitness += closeness * 100  # Up to 100 points for getting close
    
    return fitness


def eval_genomes(genomes, config):
    """Evaluate all genomes in the population"""
    eval_genomes_base(genomes, config, evaluate_genome)


class WebSocketReporter(neat.reporting.BaseReporter):
    """Reporter that sends training stats to WebSocket and auto-saves best genome"""
    
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
            
            # Update best genome info for web interface (only when new best found)
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
    """Run the NEAT training"""
    print("=" * 60)
    print("NEAT Training for Inverted Pendulum Balancing")
    print("=" * 60)
    
    # Load NEAT configuration
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        CONFIG_PATH
    )
    
    # Note: Seeding population with best genome disabled due to NEAT species tracking issues
    # The best genome is still preserved and will be compared against new genomes
    
    # Create population
    if continue_from_checkpoint and os.path.exists(CHECKPOINT_DIR):
        # Find latest checkpoint
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
    pop.add_reporter(WebSocketReporter())  # Send updates to web interface
    
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
        winner = pop.run(eval_genomes, n=999999)  # Run until manually stopped (very high limit)
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
    print("Testing best genome...")
    
    if not os.path.exists(BEST_GENOME_PATH):
        print(f"Error: No best genome found at {BEST_GENOME_PATH}")
        print("Run training first: python neat_trainer.py")
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
    
    # Create simulator (no background thread for testing)
    sim_config = create_fast_simulator()
    sim = PendulumSimulator(sim_config, start_background_thread=False)
    
    # Start pendulum at bottom (0°) - network must swing up!
    sim.set_state(
        cart_position=0.0,
        cart_velocity=0.0,
        pendulum_angle=math.radians(5),  # Slightly off from 0° (down)
        pendulum_velocity=0.0
    )
    
    # Get rail limits
    state = sim.get_state()
    if 'limit_right_pos' not in state or 'limit_left_pos' not in state:
        # Fallback: use config values
        rail_length = sim_config.rail_length_steps
    else:
        rail_length = state['limit_right_pos'] - state['limit_left_pos']
    
    print("\nRunning test... (Ctrl+C to stop)")
    print("Angle | AngVel | Position | Velocity | Output")
    print("-" * 55)
    
    try:
        while True:
            # Get current state from simulator
            state = sim.get_state()
            angle_deg = math.degrees(state['pendulum_angle']) % 360
            
            # Prepare inputs for neural network
            inputs = [
                normalize_angle(angle_deg),
                state['pendulum_velocity'] / 10.0,
                state['cart_position'] / (rail_length / 2),
                state['cart_velocity'] / MAX_SPEED,
            ]
            
            # Get network output
            output = net.activate(inputs)
            velocity = output[0] * MAX_SPEED
            velocity = max(-MAX_SPEED, min(MAX_SPEED, velocity))
            
            # Set target velocity and step physics
            sim.set_target_velocity(velocity)
            sim.step(EVAL_DT)
            
            # Print state
            state = sim.get_state()
            print(f"{angle_deg:6.1f}° | {math.degrees(state['pendulum_velocity']):6.1f} | "
                  f"{state['cart_position']:8.0f} | {state['cart_velocity']:8.0f} | {velocity:8.0f}")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nTest stopped.")
    finally:
        sim.close()


def load_best_network():
    """Load the best trained network for use in the controller"""
    if not os.path.exists(BEST_GENOME_PATH):
        return None
    
    config = neat.Config(
        neat.DefaultGenome,
        neat.DefaultReproduction,
        neat.DefaultSpeciesSet,
        neat.DefaultStagnation,
        CONFIG_PATH
    )
    
    with open(BEST_GENOME_PATH, 'rb') as f:
        genome = pickle.load(f)
    
    return neat.nn.FeedForwardNetwork.create(genome, config)


if __name__ == "__main__":
    if "--test" in sys.argv:
        test_best_genome()
    elif "--continue" in sys.argv:
        run_training(continue_from_checkpoint=True)
    else:
        run_training(continue_from_checkpoint=False)

