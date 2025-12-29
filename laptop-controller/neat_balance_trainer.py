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
from simulator import PendulumSimulator, SimulatorConfig

# Paths
CONFIG_PATH = os.path.join(os.path.dirname(__file__), 'neat_balance_config.txt')
BEST_GENOME_PATH = os.path.join(os.path.dirname(__file__), 'best_balance_genome.pkl')
CHECKPOINT_DIR = os.path.join(os.path.dirname(__file__), 'neat_balance_checkpoints')
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
TRAINING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')

# Load training parameters from JSON config
def load_training_params():
    try:
        with open(TRAINING_CONFIG_FILE, 'r') as f:
            config = json.load(f)
        balance = config.get('balance', {})
        return {
            'max_speed': balance.get('max_speed', 5000),
            'sim_steps': balance.get('sim_steps', 5000),
            'pop_size': balance.get('pop_size', 100)
        }
    except:
        return {'max_speed': 5000, 'sim_steps': 5000, 'pop_size': 100}

# Training parameters (loaded from JSON)
_params = load_training_params()
MAX_SPEED = _params['max_speed']
SIMULATION_STEPS = _params['sim_steps']
EVAL_DT = 0.02  # Evaluation timestep (50Hz)

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
        pass


def normalize_angle(angle_deg):
    """
    Normalize angle so 180° (upright) = 0, and 0°/360° (down) = ±1
    """
    # Shift so 180 becomes 0
    shifted = angle_deg - 180.0
    # Wrap to -180..180
    while shifted > 180:
        shifted -= 360
    while shifted < -180:
        shifted += 360
    # Normalize to -1..1
    return shifted / 180.0


def create_fast_simulator():
    """Create simulator config optimized for fast training"""
    config = SimulatorConfig(
        rail_length_steps=7500,
        cart_mass=0.5,
        motor_accel=500000,
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=9.81,
        damping=0.1,
    )
    return config


def evaluate_genome(genome, config):
    """
    Evaluate a single genome for BALANCE task.
    Starts pendulum near 180° (upright) with small perturbation.
    
    Fitness = total time steps the pendulum stays balanced
    """
    net = neat.nn.FeedForwardNetwork.create(genome, config)
    
    # Create simulator
    sim_config = create_fast_simulator()
    
    # State variables
    cart_position = 0.0
    cart_velocity = 0.0
    target_velocity = 0.0
    
    # Start pendulum at 180° (upright) with small random perturbation
    perturbation = random.uniform(-10, 10)  # ±10 degrees
    pendulum_angle = math.radians(180 + perturbation)
    pendulum_velocity = random.uniform(-0.5, 0.5)  # Small initial velocity
    
    # Limits
    limit_left = -sim_config.rail_length_steps / 2
    limit_right = sim_config.rail_length_steps / 2
    rail_length = sim_config.rail_length_steps
    
    fitness = 0.0
    
    for step in range(SIMULATION_STEPS):
        # Get current state as neural network inputs
        angle_deg = math.degrees(pendulum_angle) % 360
        
        inputs = [
            normalize_angle(angle_deg),           # Angle from upright (-1 to 1)
            pendulum_velocity / 10.0,             # Angular velocity (normalized)
            cart_position / (rail_length / 2),    # Position (normalized)
            cart_velocity / MAX_SPEED,            # Cart velocity (normalized)
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

        
        # Simple fitness: +1 for each step within 30° of upright
        if angle_from_up < 0.167:  # 30°/180° ≈ 0.167
            fitness += 1.0
        
        # Stop early if fell too far 
        if angle_from_up > 0.5: 
            print(f"Fell too far: {angle_deg}")
            break
        
        # Stop if hit limit
        if hit_limit:
            fitness -= 100  # Penalty for hitting limit
            break
    
    return fitness


def eval_genomes(genomes, config):
    """Evaluate all genomes in the population"""
    for genome_id, genome in genomes:
        genome.fitness = evaluate_genome(genome, config)


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
        send_training_update()


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
    
    # Run simulation
    sim_config = create_fast_simulator()
    cart_position = 0.0
    cart_velocity = 0.0
    pendulum_angle = math.radians(180 + 5)  # Start 5° off vertical
    pendulum_velocity = 0.0
    
    limit_left = -sim_config.rail_length_steps / 2
    limit_right = sim_config.rail_length_steps / 2
    rail_length = sim_config.rail_length_steps
    
    print("\nRunning simulation (Ctrl+C to stop)...")
    print("Time(s) | Angle(°) | Velocity | Cart Pos")
    print("-" * 45)
    
    try:
        for step in range(SIMULATION_STEPS * 2):
            angle_deg = math.degrees(pendulum_angle) % 360
            
            inputs = [
                normalize_angle(angle_deg),
                pendulum_velocity / 10.0,
                cart_position / (rail_length / 2),
                cart_velocity / MAX_SPEED,
            ]
            
            output = net.activate(inputs)
            target_velocity = output[0] * MAX_SPEED
            target_velocity = max(-MAX_SPEED, min(MAX_SPEED, target_velocity))
            
            # Physics update
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
            if cart_position <= limit_left:
                cart_position = limit_left
                if cart_velocity < 0:
                    cart_velocity = 0
            if cart_position >= limit_right:
                cart_position = limit_right
                if cart_velocity > 0:
                    cart_velocity = 0
            
            # Pendulum physics
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
            
            while pendulum_angle < 0:
                pendulum_angle += 2 * math.pi
            while pendulum_angle >= 2 * math.pi:
                pendulum_angle -= 2 * math.pi
            
            # Print every 0.5 seconds
            if step % 25 == 0:
                time_s = step * dt
                angle_display = math.degrees(pendulum_angle)
                print(f"{time_s:7.1f} | {angle_display:8.1f} | {target_velocity:8.0f} | {cart_position:8.0f}")
            
            time.sleep(dt / 10)  # Speed up visualization
            
    except KeyboardInterrupt:
        print("\nTest stopped.")


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

