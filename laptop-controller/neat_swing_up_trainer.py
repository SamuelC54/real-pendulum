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
import asyncio
# No websockets needed - using file-based communication
import threading
from simulator import PendulumSimulator, SimulatorConfig

# File-based communication for training progress
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
training_stats = {
    "generation": 0,
    "best_fitness": 0,
    "avg_fitness": 0,
    "species_count": 0,
    "population_size": 0,
    "running": True
}

def send_training_update():
    """Write training stats to file for controller to read"""
    try:
        with open(TRAINING_STATUS_FILE, 'w') as f:
            json.dump(training_stats, f)
    except:
        pass  # Just skip if file write fails

# Training parameters
MAX_SPEED = 9000          # Max cart velocity for training
SIMULATION_STEPS = 10000    # Steps per evaluation (at 50Hz = 40 seconds)
EVAL_DT = 0.02             # Evaluation timestep (50Hz)

# Paths
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'neat_swing_up_config.txt')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'neat_swing_up_checkpoints')
BEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), 'best_swing_up_genome.pkl')


def normalize_angle(angle_deg):
    """Normalize angle so 180° (upright) = 0, range [-1, 1]"""
    # angle_deg: 0=down, 180=up
    # We want: 180=0 (balanced), 0 or 360 = ±1 (fallen)
    diff = angle_deg - 180.0
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff / 180.0  # -1 to 1


def create_fast_simulator():
    """Create a simulator optimized for fast training"""
    config = SimulatorConfig(
        rail_length_steps=7500,
        motor_accel=500000,  # Fast acceleration for training
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=9.81,
        damping=0.1,  # Some damping but not too much
    )
    return config


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
    
    # State variables
    cart_position = 0.0
    cart_velocity = 0.0
    
    # Start pendulum at BOTTOM (0°) with small random offset
    pendulum_angle = math.radians(random.uniform(-5, 5))  # Near 0° (down)
    pendulum_velocity = 0.0
    
    # Limits
    limit_left = -sim_config.rail_length_steps / 2
    limit_right = sim_config.rail_length_steps / 2
    rail_length = sim_config.rail_length_steps
    
    # Tracking variables
    max_velocity_used = 0.0  # Track maximum velocity used
    total_velocity_used = 0.0  # Track cumulative velocity usage
    reached_top = False
    step_reached_top = SIMULATION_STEPS  # When did it reach top
    
    for step in range(SIMULATION_STEPS):
        angle_deg = math.degrees(pendulum_angle) % 360
        
        inputs = [
            normalize_angle(angle_deg),          # Angle from upright (-1 to 1)
            pendulum_velocity / 10.0,            # Angular velocity (normalized)
            cart_position / (rail_length / 2),   # Position (normalized)
            cart_velocity / MAX_SPEED,           # Cart velocity (normalized)
        ]
        
        output = net.activate(inputs)
        target_velocity = output[0] * MAX_SPEED
        target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
        
        # Track velocity usage
        total_velocity_used += abs(target_velocity)
        if abs(target_velocity) > max_velocity_used:
            max_velocity_used = abs(target_velocity)
        
        # --- Physics simulation ---
        dt = EVAL_DT
        
        velocity_error = target_velocity - cart_velocity
        max_accel_change = sim_config.motor_accel * dt
        
        if abs(velocity_error) < max_accel_change:
            cart_velocity = target_velocity
        else:
            cart_velocity += math.copysign(max_accel_change, velocity_error)
        
        cart_accel = velocity_error / dt if dt > 0 else 0
        cart_accel = max(-sim_config.motor_accel, min(sim_config.motor_accel, cart_accel))
        
        cart_position += cart_velocity * dt
        
        # Check limits
        hit_limit = False
        if cart_position <= limit_left:
            cart_position = limit_left
            if cart_velocity < 0:
                cart_velocity = 0
            hit_limit = True
        if cart_position >= limit_right:
            cart_position = limit_right
            if cart_velocity > 0:
                cart_velocity = 0
            hit_limit = True
        
        # Pendulum dynamics
        steps_per_meter = 100000
        cart_accel_ms2 = cart_accel / steps_per_meter
        
        L = sim_config.pendulum_length
        g = sim_config.gravity
        c = sim_config.damping
        
        gravity_term = -(g / L) * math.sin(pendulum_angle)
        coupling_term = (cart_accel_ms2 / L) * math.cos(pendulum_angle)
        damping_term = c * pendulum_velocity
        
        angular_accel = gravity_term + coupling_term - damping_term
        pendulum_velocity += angular_accel * dt
        pendulum_angle += pendulum_velocity * dt
        
        # Normalize angle
        while pendulum_angle < 0:
            pendulum_angle += 2 * math.pi
        while pendulum_angle >= 2 * math.pi:
            pendulum_angle -= 2 * math.pi
        
        # Check if reached top (within 10° of 180°)
        angle_deg = math.degrees(pendulum_angle)
        angle_from_up = abs(normalize_angle(angle_deg))
        
        if angle_from_up < 0.056 and not reached_top:  # Within ~10° of 180°
            reached_top = True
            step_reached_top = step
        
        # Stop early if hit limit
        if hit_limit:
            break
    
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
        # Check final angle
        final_angle_deg = math.degrees(pendulum_angle) % 360
        angle_from_up = abs(normalize_angle(final_angle_deg))
        closeness = 1.0 - angle_from_up  # 0 at bottom, 1 at top
        fitness += closeness * 100  # Up to 100 points for getting close
    
    return fitness


def eval_genomes(genomes, config):
    """Evaluate all genomes in the population"""
    for genome_id, genome in genomes:
        genome.fitness = evaluate_genome(genome, config)


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
        send_training_update()


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
    
    # Run interactive test with the full simulator
    sim = PendulumSimulator()
    
    # Start pendulum at bottom (0°) - network must swing up!
    sim.pendulum_angle = math.radians(5)  # Slightly off from 0° (down)
    
    print("\nRunning test... (Ctrl+C to stop)")
    print("Angle | AngVel | Position | Velocity | Output")
    print("-" * 55)
    
    try:
        while True:
            # Get state
            angle_deg = math.degrees(sim.pendulum_angle) % 360
            
            inputs = [
                normalize_angle(angle_deg),
                sim.pendulum_velocity / 10.0,
                sim.cart_position / (sim.config.rail_length_steps / 2),
                sim.cart_velocity / MAX_SPEED,
            ]
            
            # Get network output
            output = net.activate(inputs)
            velocity = output[0] * MAX_SPEED
            
            # Send to simulator
            sim.target_velocity = velocity
            
            # Print state
            print(f"{angle_deg:6.1f}° | {math.degrees(sim.pendulum_velocity):6.1f} | "
                  f"{sim.cart_position:8.0f} | {sim.cart_velocity:8.0f} | {velocity:8.0f}")
            
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

