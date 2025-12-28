"""
NEAT-Python Trainer for Inverted Pendulum Balancing

Uses neuroevolution to train a neural network that keeps the pendulum
balanced upright (around 180°) by controlling cart velocity.

Inputs (4):
  - Pendulum angle (normalized, 180° = 0)
  - Pendulum angular velocity (normalized)
  - Cart position (normalized)
  - Cart velocity (normalized)

Output (1):
  - Cart velocity command (-1 to 1, scaled to max speed)

Usage:
  python neat_trainer.py              # Train new network
  python neat_trainer.py --continue   # Continue from checkpoint
  python neat_trainer.py --test       # Test best genome
"""

import os
import sys
import math
import pickle
import neat
import time
from simulator import PendulumSimulator, SimulatorConfig

# Training parameters
MAX_SPEED = 100000          # Max cart velocity for training
SIMULATION_STEPS = 2000    # Steps per evaluation (at 50Hz = 40 seconds)
EVAL_DT = 0.02             # Evaluation timestep (50Hz)

# Paths
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'neat_config.txt')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'neat_checkpoints')
BEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), 'best_genome.pkl')


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


def evaluate_genome(genome, config, visualize=False):
    """
    Evaluate a single genome by running it in the simulator.
    
    Fitness:
    - Exponential reward based on closeness to 180° (upright)
    - Big penalty for touching limits
    
    Inputs (6):
    - Angle from upright (normalized)
    - Angular velocity (normalized)
    - Cart position (normalized)
    - Cart velocity (normalized)
    - Distance to left limit (normalized, 0=at limit, 1=far)
    - Distance to right limit (normalized, 0=at limit, 1=far)
    """
    # Create neural network from genome
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    
    # Create simulator with fast settings
    sim_config = create_fast_simulator()
    
    # State variables
    cart_position = 0.0
    cart_velocity = 0.0
    target_velocity = 0.0
    
    # Start pendulum at BOTTOM (0°) - must swing up!
    pendulum_angle = math.radians(hash(str(genome.key)) % 10 - 5)  # Near 0° (down)
    pendulum_velocity = 0.0
    
    # Limits
    limit_left = -sim_config.rail_length_steps / 2
    limit_right = sim_config.rail_length_steps / 2
    rail_length = sim_config.rail_length_steps
    
    fitness = 0.0
    reached_top = False  # Track if pendulum reached near 180°
    
    for step in range(SIMULATION_STEPS):
        # Get current state as neural network inputs
        angle_deg = math.degrees(pendulum_angle) % 360
        
        # Distance to limits (0 = at limit, 1 = at opposite limit)
        dist_to_left = (cart_position - limit_left) / rail_length
        dist_to_right = (limit_right - cart_position) / rail_length
        
        inputs = [
            normalize_angle(angle_deg),          # Angle from upright (-1 to 1)
            pendulum_velocity / 10.0,            # Angular velocity (normalized)
            cart_position / (rail_length / 2),   # Position (normalized)
            cart_velocity / MAX_SPEED,           # Velocity (normalized)
            dist_to_left,                        # Distance to left limit (0-1)
            dist_to_right,                       # Distance to right limit (0-1)
        ]
        
        # Get network output
        output = net.activate(inputs)
        target_velocity = output[0] * MAX_SPEED
        target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
        
        # --- Physics simulation ---
        dt = EVAL_DT
        
        # Cart dynamics
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
        
        # --- Fitness calculation ---
        angle_deg = math.degrees(pendulum_angle)
        angle_from_up = abs(normalize_angle(angle_deg))  # 0 = at 180°, 1 = at 0°
        
        # Check if reached near 180° (within ±3°, which is ~0.017 normalized)
        if angle_from_up < 0.017:  # Within ~3° of 180°
            reached_top = True
        
        # If reached top and fell back down (past 90° from top), reset score
        if reached_top and angle_from_up > 0.5:  # Fell past 90° from upright
            fitness = 0.0
            reached_top = False  # Reset to allow another attempt
        
        # Exponential reward: more points when closer to 180°
        # closeness = 1 when at 180°, 0 when at 0°
        closeness = 1.0 - angle_from_up
        reward = closeness ** 2  # Exponential (squared) - max 1.0 when at 180°
        fitness += reward
        
        # Big penalty for hitting limits
        if hit_limit:
            fitness -= 10.0
    
    return fitness


def eval_genomes(genomes, config):
    """Evaluate all genomes in the population"""
    for genome_id, genome in genomes:
        genome.fitness = evaluate_genome(genome, config)


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
        winner = pop.run(eval_genomes, n=300)  # Run for up to 300 generations
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

