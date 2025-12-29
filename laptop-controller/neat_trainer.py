"""
NEAT-Python Trainer for Inverted Pendulum Balancing

Uses neuroevolution to train a neural network that keeps the pendulum
balanced upright (around 180°) by controlling cart velocity.

Inputs (4):
  - Angle from upright (normalized: 180° = 0, 0°/360° = ±1)
  - Angular velocity (normalized)
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
import random
import neat
import time
import json
import asyncio
import websockets
import threading
from simulator import PendulumSimulator, SimulatorConfig

# WebSocket for sending training progress to web interface
WS_URL = "ws://127.0.0.1:8765"
ws_loop = None
ws_connection = None
ws_connected = False
training_stats = {
    "generation": 0,
    "best_fitness": 0,
    "avg_fitness": 0,
    "species_count": 0,
    "population_size": 0,
    "fitness_history": []
}

def send_training_update():
    """Send training stats to web interface via WebSocket"""
    global ws_connection, ws_loop, ws_connected
    if not ws_connected or not ws_connection or not ws_loop:
        return
    try:
        msg = json.dumps({"type": "TRAINING", **training_stats})
        future = asyncio.run_coroutine_threadsafe(ws_connection.send(msg), ws_loop)
        future.result(timeout=0.5)
    except:
        pass  # Just skip this update if it fails

async def ws_runner():
    """Run WebSocket connection - connect once and keep alive"""
    global ws_connection, ws_connected
    
    # Try to connect once
    try:
        ws_connection = await websockets.connect(WS_URL, ping_interval=30, ping_timeout=20)
        ws_connected = True
        print(f"Connected to WebSocket at {WS_URL}")
        
        # Consume incoming messages to keep connection alive
        # (controller broadcasts state to all clients)
        async for message in ws_connection:
            # Just discard incoming messages - we only send, not receive
            pass
                
    except websockets.exceptions.ConnectionClosed:
        print("WebSocket connection closed by server")
        ws_connected = False
    except Exception as e:
        print(f"WebSocket not available - training progress won't be shown")
        ws_connected = False

def start_ws_thread():
    """Run WebSocket event loop in background thread"""
    global ws_loop
    ws_loop = asyncio.new_event_loop()
    asyncio.set_event_loop(ws_loop)
    try:
        ws_loop.run_until_complete(ws_runner())
    except:
        pass

# Training parameters
MAX_SPEED = 10000          # Max cart velocity for training
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


eval_count = 0  # Global counter for logging

def evaluate_genome(genome, config, visualize=False):
    global eval_count
    """
    Evaluate a single genome by running it in the simulator.
    
    Fitness:
    - Exponential reward based on closeness to 180° (upright)
    - Big penalty for touching limits
    
    Inputs (4):
    - Angle from upright (normalized: 180° = 0, 0°/360° = ±1)
    - Angular velocity (normalized)
    - Cart position (normalized)
    - Cart velocity (normalized)
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
    ### Start pendulum at random angle (0-360°)
    ##pendulum_angle = math.radians(random.uniform(0, 360))
    pendulum_velocity = 0.0
    
    # Limits
    limit_left = -sim_config.rail_length_steps / 2
    limit_right = sim_config.rail_length_steps / 2
    rail_length = sim_config.rail_length_steps
    
    fitness = 0.0
    reached_top = False  # Track if pendulum reached near 180°
    net_rotation = 0.0  # Track net rotation (positive = clockwise, negative = counter)
    last_angle_rad = pendulum_angle
    
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
            cart_velocity / MAX_SPEED,           # Cart velocity (normalized)
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
        
        # Track net rotation using angular velocity
        # This properly accounts for direction and doesn't double-count oscillations
        net_rotation += pendulum_velocity * dt
        
        angle_from_up = abs(normalize_angle(angle_deg))  # 0 = at 180°, 1 = at 0°
        
        # # Check if reached near 180° 
        # if angle_from_up < 0.1:  # Within 10° of 180°
        #     reached_top = True
        
        # # If reached top and fell back down (past 90° from top), reset score
        # if reached_top and angle_from_up > 0.5:  # Fell past 90° from upright
        #     fitness = 0
        #     reached_top = False  # Reset to allow another attempt
        
        # Exponential reward: more points when closer to 180°
        # closeness = 1 when at 180°, 0 when at 0°
        closeness = 1.0 - angle_from_up
        reward = closeness ** 2  # Exponential (squared) - max 1.0 when at 180°
        fitness += reward
        
        # Big penalty for hitting limits
        if hit_limit:
            fitness -= 10.0

        # Penalty for high angular velocity (encourages smooth control)
        # fitness -= 0.02 * abs(pendulum_velocity)

        # Penalty for high cart velocity (encourages smooth control)
        # fitness -= 0.00001 * abs(cart_velocity)
    
    # Count full rotations (2*pi radians = 1 full loop)
    full_rotations = abs(net_rotation) / (2 * math.pi)
    
    # Penalty for doing more than 2 full rotations in either direction
    if full_rotations > 2:
        fitness -= 50.0 * (full_rotations - 2)
    
    # Debug: log rotation
    # print(f"Genome {genome.key}: rotations={full_rotations:.1f}, fitness={fitness:.1f}")
    
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
        
        # Keep last 100 fitness values for graph
        training_stats["fitness_history"].append(training_stats["best_fitness"])
        if len(training_stats["fitness_history"]) > 100:
            training_stats["fitness_history"] = training_stats["fitness_history"][-100:]
        
        # Send to WebSocket
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
    
    # Try to connect to WebSocket for live updates
    ws_thread = threading.Thread(target=start_ws_thread, daemon=True)
    ws_thread.start()
    time.sleep(0.5)  # Give time to connect
    
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

