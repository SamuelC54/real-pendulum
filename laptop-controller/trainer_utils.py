"""
Shared utility functions for NEAT trainers
"""

import os
import json
import math
from simulator import SimulatorConfig

# Constants
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
TRAINING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')


def normalize_angle(angle_deg):
    """
    Normalize angle so 180° (upright) = 0, and 0°/360° (down) = ±1
    
    Args:
        angle_deg: Angle in degrees (0-360)
    
    Returns:
        Normalized angle in range [-1, 1] where:
        - 180° (upright) = 0
        - 0° or 360° (down) = ±1
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
        motor_accel=500000,  # Fast acceleration for training
        pendulum_length=0.3,
        pendulum_mass=0.1,
        gravity=9.81,
        damping=0.1,  # Some damping but not too much
    )
    return config


def load_training_params(trainer_type: str = "swing_up"):
    """
    Load training parameters from JSON config file
    
    Args:
        trainer_type: Either "swing_up" or "balance"
    
    Returns:
        Dict with max_speed, sim_steps, and pop_size
    """
    try:
        with open(TRAINING_CONFIG_FILE, 'r') as f:
            config = json.load(f)
        trainer_config = config.get(trainer_type, {})
        defaults = {
            'swing_up': {'max_speed': 9000, 'sim_steps': 2000, 'pop_size': 100},
            'balance': {'max_speed': 5000, 'sim_steps': 5000, 'pop_size': 100}
        }
        default = defaults.get(trainer_type, defaults['swing_up'])
        return {
            'max_speed': trainer_config.get('max_speed', default['max_speed']),
            'sim_steps': trainer_config.get('sim_steps', default['sim_steps']),
            'pop_size': trainer_config.get('pop_size', default['pop_size'])
        }
    except:
        # throw if not found
        raise Exception(f"Training parameters not found for {trainer_type}")


def send_training_update(training_stats: dict):
    """
    Write training stats to file for controller to read
    
    Args:
        training_stats: Dictionary with training statistics
    """
    try:
        with open(TRAINING_STATUS_FILE, 'w') as f:
            json.dump(training_stats, f)
    except:
        pass  # Just skip if file write fails


def eval_genomes(genomes, config, evaluate_func):
    """
    Evaluate all genomes in the population
    
    Args:
        genomes: List of (genome_id, genome) tuples
        config: NEAT config object
        evaluate_func: Function to evaluate a single genome (genome, config) -> fitness
    """
    for genome_id, genome in genomes:
        genome.fitness = evaluate_func(genome, config)

