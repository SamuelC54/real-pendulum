"""
Inverted Pendulum - Velocity Streaming Controller

All control logic runs here in Python.
Arduino is a simple sensor/actuator interface.

Communication:
- Serial to Arduino: Velocity commands "V1234\n"
- Serial from Arduino: "A:180.5,P:1000,LL:0,LR:1,V:1234\n"
- WebSocket to Web UI: JSON state updates

Usage:
  python controller.py         # Connect to real Arduino
  python controller.py --sim   # Use physics simulator (no hardware needed)
"""

import serial
import asyncio
import websockets
import json
import time
import math
import subprocess
import os
import sys
import threading
from dataclasses import dataclass, asdict
from typing import Optional
from collections import deque
from concurrent.futures import ThreadPoolExecutor

# Check for simulator mode
SIMULATOR_MODE = "--sim" in sys.argv or "-s" in sys.argv

# NEAT neural network for balance mode
neat_network = None  # Loaded on startup if available

# ============ CONFIGURATION ============
SERIAL_PORT = "COM3"
BAUD = 115200
CONTROL_RATE = 100  # Hz - control loop frequency

# Arduino upload settings
BOARD = "arduino:avr:uno"
SKETCH_PATH = os.path.join(os.path.dirname(__file__), "..", "arduino-code", "arduino-code.ino")

# WebSocket server
WS_HOST = "0.0.0.0"  # Bind to all interfaces
WS_PORT = 8765

# ============ STATE ============
@dataclass
class PendulumState:
    angle: float = 180.0           # degrees, 0/360 = up, 180 = down
    angular_velocity: float = 0.0  # degrees/second
    position: int = 0              # steps from zero
    velocity: int = 0              # current motor velocity
    limit_left: bool = False
    limit_right: bool = False
    mode: str = "idle"             # idle, manual_left, manual_right, oscillate, neat_swing_up_only, neat_balance_only, homing
    connected: bool = False
    # Homing state
    homing_phase: int = 0          # 0=not homing, 1=going right, 2=going left, 3=going to center
    limit_left_pos: int = 0        # Position at left limit
    limit_right_pos: int = 0       # Position at right limit
    home_speed: int = 2000         # Speed for homing

@dataclass
class ControlConfig:
    # Manual mode
    manual_speed: int = 1000
    manual_accel: int = 50000
    
    # Oscillate mode
    oscillate_speed: int = 3000
    oscillate_period: float = 2.0  # seconds per cycle
    
    # NEAT balance mode
    neat_max_speed: int = 100000  # Max speed for NEAT controller

# Global state
state = PendulumState()
config = ControlConfig()
ws_clients: set = set()
serial_port: Optional[serial.Serial] = None
executor = ThreadPoolExecutor(max_workers=1)  # For serial I/O

# For angular velocity calculation
last_angle = 180.0
last_angle_time = time.perf_counter()

# ============ NEAT NETWORK ============
best_genome_info = None  # Stores info about the best saved genome
neat_config_values = None  # Stores current NEAT config values
balance_network = None  # Separate network for balance-only mode
balance_genome_info = None  # Info about balance genome

NEAT_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')

def load_training_config():
    """Load training config from JSON file"""
    try:
        with open(NEAT_CONFIG_FILE, 'r') as f:
            return json.load(f)
    except:
        # Default config
        return {
            "swing_up": {"pop_size": 100, "max_speed": 9000, "sim_steps": 2000},
            "balance": {"pop_size": 100, "max_speed": 5000, "sim_steps": 5000}
        }

def save_training_config(config: dict):
    """Save training config to JSON file"""
    try:
        with open(NEAT_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
    except Exception as e:
        print(f"Error saving config: {e}")

def get_neat_config(trainer_type: str = "swing_up"):
    """Get config for a specific trainer type"""
    global neat_config_values
    config = load_training_config()
    
    # Map tab names to config keys
    key = "swing_up" if trainer_type in ["swing-up", "swing_up"] else "balance"
    trainer_config = config.get(key, {"pop_size": 100, "max_speed": 9000, "sim_steps": 2000})
    
    neat_config_values = {
        "pop_size": trainer_config.get("pop_size", 100),
        "max_speed": trainer_config.get("max_speed", 9000),
        "sim_steps": trainer_config.get("sim_steps", 2000)
    }
    
    print(f"NEAT Config ({key}): pop={neat_config_values['pop_size']}, max_speed={neat_config_values['max_speed']}, sim_steps={neat_config_values['sim_steps']}", flush=True)
    
    return neat_config_values

def load_neat_network():
    """Load the trained NEAT network if available"""
    global neat_network, best_genome_info
    
    genome_path = os.path.join(os.path.dirname(__file__), 'best_swing_up_genome.pkl')
    config_path = os.path.join(os.path.dirname(__file__), 'neat_swing_up_config.txt')
    
    if not os.path.exists(genome_path):
        print("NEAT: No trained network found (best_genome.pkl)")
        best_genome_info = None
        return False
    
    if not os.path.exists(config_path):
        print("NEAT: Config file not found (neat_swing_up_config.txt)")
        best_genome_info = None
        return False
    
    try:
        import neat
        import pickle
        
        # Load NEAT config
        neat_config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_path
        )
        
        # Load best genome
        with open(genome_path, 'rb') as f:
            genome = pickle.load(f)
        
        # Create network
        neat_network = neat.nn.FeedForwardNetwork.create(genome, neat_config)
        
        # Extract genome info for display
        num_nodes = len(genome.nodes)
        num_connections = len([c for c in genome.connections.values() if c.enabled])
        num_inputs = len(neat_network.input_nodes)
        num_outputs = len(neat_network.output_nodes)
        
        # Get file modification time
        import datetime
        mod_time = os.path.getmtime(genome_path)
        mod_datetime = datetime.datetime.fromtimestamp(mod_time)
        
        best_genome_info = {
            "fitness": genome.fitness,
            "nodes": num_nodes,
            "connections": num_connections,
            "inputs": num_inputs,
            "outputs": num_outputs,
            "saved_at": mod_datetime.strftime("%Y-%m-%d %H:%M")
        }
        
        print(f"NEAT: Loaded trained network (fitness: {genome.fitness:.1f}, nodes: {num_nodes}, conns: {num_connections})")
        return True
        
    except Exception as e:
        print(f"NEAT: Failed to load network: {e}")
        best_genome_info = None
        return False

def load_balance_network():
    """Load the trained balance-only NEAT network if available"""
    global balance_network, balance_genome_info
    
    genome_path = os.path.join(os.path.dirname(__file__), 'best_balance_genome.pkl')
    config_path = os.path.join(os.path.dirname(__file__), 'neat_balance_config.txt')
    
    if not os.path.exists(genome_path):
        print("NEAT Balance: No trained network found (best_balance_genome.pkl)")
        balance_genome_info = None
        return False
    
    if not os.path.exists(config_path):
        print("NEAT Balance: Config file not found (neat_balance_config.txt)")
        balance_genome_info = None
        return False
    
    try:
        import neat
        import pickle
        
        # Load NEAT config
        neat_config = neat.Config(
            neat.DefaultGenome,
            neat.DefaultReproduction,
            neat.DefaultSpeciesSet,
            neat.DefaultStagnation,
            config_path
        )
        
        # Load best genome
        with open(genome_path, 'rb') as f:
            genome = pickle.load(f)
        
        # Create network
        balance_network = neat.nn.FeedForwardNetwork.create(genome, neat_config)
        
        # Extract genome info
        num_nodes = len(genome.nodes)
        num_connections = len([c for c in genome.connections.values() if c.enabled])
        
        import datetime
        mod_time = os.path.getmtime(genome_path)
        mod_datetime = datetime.datetime.fromtimestamp(mod_time)
        
        balance_genome_info = {
            "fitness": genome.fitness,
            "nodes": num_nodes,
            "connections": num_connections,
            "saved_at": mod_datetime.strftime("%Y-%m-%d %H:%M")
        }
        
        print(f"NEAT Balance: Loaded network (fitness: {genome.fitness:.1f})")
        return True
        
    except Exception as e:
        print(f"NEAT Balance: Failed to load network: {e}")
        balance_genome_info = None
        return False

def normalize_angle_for_neat(angle_deg):
    """Normalize angle so 180° (upright) = 0, range [-1, 1]"""
    # In our system: 0° = down, 180° = up
    # We want: 180° = 0 (balanced), 0° or 360° = ±1 (fallen)
    diff = angle_deg - 180.0
    if diff > 180:
        diff -= 360
    elif diff < -180:
        diff += 360
    return diff / 180.0

# ============ SERIAL COMMUNICATION ============
def connect_serial():
    global serial_port
    
    if SIMULATOR_MODE:
        # Use physics simulator instead of real serial
        from simulator import PendulumSimulator
        serial_port = PendulumSimulator()
        state.connected = True
        print("Connected to SIMULATOR")
        return True
    
    try:
        serial_port = serial.Serial(SERIAL_PORT, BAUD, timeout=0.01, write_timeout=0.1)
        state.connected = True
        print(f"Connected to {SERIAL_PORT}")
        return True
    except Exception as e:
        print(f"Serial connection failed: {e}")
        state.connected = False
        return False

last_debug_velocity = [0]  # Use list to allow modification in nested function

def send_velocity(velocity: int):
    """Send velocity command to Arduino"""
    if serial_port and serial_port.is_open:
        try:
            cmd = f"V{velocity}\n"
            serial_port.write(cmd.encode())
            # Debug: print when velocity changes significantly
            if abs(velocity - last_debug_velocity[0]) > 100:
                print(f"Sending velocity: {velocity}", flush=True)
                last_debug_velocity[0] = velocity
        except serial.SerialTimeoutException:
            pass  # Ignore write timeouts, next command will resync
        except Exception as e:
            print(f"Send error: {e}", flush=True)

def send_stop():
    """Send stop command"""
    if serial_port and serial_port.is_open:
        try:
            serial_port.write(b"S\n")
        except serial.SerialTimeoutException:
            pass
        except Exception as e:
            print(f"Send error: {e}", flush=True)

def send_zero():
    """Zero the encoder and position"""
    if serial_port and serial_port.is_open:
        try:
            # Adjust limit positions relative to new zero
            current_pos = state.position
            if state.limit_left_pos != 0:
                state.limit_left_pos -= current_pos
            if state.limit_right_pos != 0:
                state.limit_right_pos -= current_pos
            serial_port.write(b"Z\n")
            print(f"Zeroed. Limits now: L={state.limit_left_pos}, R={state.limit_right_pos}", flush=True)
        except serial.SerialTimeoutException:
            pass
        except Exception as e:
            print(f"Send error: {e}", flush=True)

def parse_sensor_data(line: str) -> bool:
    """Parse sensor data from Arduino: A:180.5,P:1000,LL:0,LR:1,V:1234"""
    global last_angle, last_angle_time
    
    try:
        parts = dict(p.split(":") for p in line.split(","))
        
        new_angle = float(parts.get("A", state.angle))
        now = time.perf_counter()
        dt = now - last_angle_time
        
        # Calculate angular velocity with wrapping
        if dt > 0:
            angle_diff = new_angle - last_angle
            # Handle wrap-around
            if angle_diff > 180:
                angle_diff -= 360
            elif angle_diff < -180:
                angle_diff += 360
            state.angular_velocity = angle_diff / dt
        
        last_angle = new_angle
        last_angle_time = now
        
        state.angle = new_angle
        state.position = int(parts.get("P", state.position))
        state.limit_left = parts.get("LL", "0") == "1"
        state.limit_right = parts.get("LR", "0") == "1"
        state.velocity = int(float(parts.get("V", state.velocity)))
        
        return True
    except Exception as e:
        return False

def read_serial():
    """Read and parse serial data from Arduino"""
    if not serial_port or not serial_port.is_open:
        return
    
    while serial_port.in_waiting:
        try:
            line = serial_port.readline().decode().strip()
            if line.startswith("A:"):
                parse_sensor_data(line)
            elif line:
                print(f"Arduino: {line}")
        except:
            pass

# ============ CONTROL ALGORITHMS ============
def compute_velocity() -> int:
    """Compute desired velocity based on current mode"""
    
    if state.mode == "idle":
        return 0
    
    elif state.mode == "manual_left":
        if state.limit_left:  # Left limit blocks moving left
            return 0
        return -config.manual_speed  # Negative = left
    
    elif state.mode == "manual_right":
        if state.limit_right:  # Right limit blocks moving right
            return 0
        return config.manual_speed  # Positive = right
    
    elif state.mode == "oscillate":
        return compute_oscillate()
    
    elif state.mode == "neat_swing_up_only":
        return compute_neat_swing_up_only()
    
    elif state.mode == "neat_balance_only":
        return compute_neat_balance_only()
    
    elif state.mode == "homing":
        return compute_homing()
    
    return 0

def compute_oscillate() -> int:
    """Simple sinusoidal oscillation"""
    t = time.perf_counter()
    phase = (t % config.oscillate_period) / config.oscillate_period * 2 * math.pi
    velocity = int(config.oscillate_speed * math.sin(phase))
    
    # Respect limits
    if state.limit_left and velocity < 0:
        return 0
    if state.limit_right and velocity > 0:
        return 0
    
    return velocity

def compute_neat_swing_up_only() -> int:
    """
    Use trained NEAT neural network to balance the pendulum.
    
    Network inputs (4):
      - Normalized angle from upright (180° = 0, 0° = ±1)
      - Angular velocity (normalized)
      - Cart position (normalized)
      - Cart velocity (normalized)
    
    Network output (1):
      - Cart velocity (-1 to 1, scaled by neat_max_speed)
    """
    if neat_network is None:
        print("NEAT network not loaded! Going to idle.")
        state.mode = "idle"
        return 0
    
    # Get rail limits for normalization
    rail_half = max(abs(state.limit_left_pos), abs(state.limit_right_pos), 3750)
    
    # Prepare inputs for neural network (4 inputs)
    inputs = [
        normalize_angle_for_neat(state.angle),           # Angle from upright
        state.angular_velocity / 500.0,                  # Angular velocity (normalized)
        state.position / rail_half,                      # Position (normalized)
        state.velocity / config.neat_max_speed,          # Cart velocity (normalized)
    ]
    
    # Get network output
    output = neat_network.activate(inputs)
    velocity = int(output[0] * config.neat_max_speed)
    
    # Clamp to max speed
    velocity = max(-config.neat_max_speed, min(config.neat_max_speed, velocity))
    
    # Respect limits
    if state.limit_left and velocity < 0:
        velocity = 0
    if state.limit_right and velocity > 0:
        velocity = 0
    
    return velocity

def compute_neat_balance_only() -> int:
    """
    Use the balance-only NEAT network (trained from upright position).
    Uses lower speed since it's only for fine balance adjustments.
    """
    if balance_network is None:
        print("Balance network not loaded! Going to idle.")
        state.mode = "idle"
        return 0
    
    # Get rail limits for normalization
    rail_half = max(abs(state.limit_left_pos), abs(state.limit_right_pos), 3750)
    
    # Balance network uses lower max speed (5000)
    balance_max_speed = 5000
    
    # Prepare inputs for neural network (4 inputs)
    inputs = [
        normalize_angle_for_neat(state.angle),           # Angle from upright
        state.angular_velocity / 10.0,                   # Angular velocity (normalized, same as trainer)
        state.position / rail_half,                      # Position (normalized)
        state.velocity / balance_max_speed,              # Cart velocity (normalized)
    ]
    
    # Get network output
    output = balance_network.activate(inputs)
    velocity = int(output[0] * balance_max_speed)
    
    # Clamp to max speed
    velocity = max(-balance_max_speed, min(balance_max_speed, velocity))
    
    # Respect limits
    if state.limit_left and velocity < 0:
        velocity = 0
    if state.limit_right and velocity > 0:
        velocity = 0
    
    return velocity

def compute_homing() -> int:
    """
    Homing sequence:
    Phase 1: Go right until hitting right limit
    Phase 2: Go left until hitting left limit
    Phase 3: Go to center and set zero
    """
    if state.homing_phase == 1:
        # Phase 1: Going right to find right limit
        if state.limit_right:
            state.limit_right_pos = state.position
            state.homing_phase = 2
            print(f"Homing: Found right limit at {state.limit_right_pos}", flush=True)
            return 0
        return state.home_speed  # Positive = right
    
    elif state.homing_phase == 2:
        # Phase 2: Going left to find left limit
        if state.limit_left:
            state.limit_left_pos = state.position
            state.homing_phase = 3
            print(f"Homing: Found left limit at {state.limit_left_pos}", flush=True)
            return 0
        return -state.home_speed  # Negative = left
    
    elif state.homing_phase == 3:
        # Phase 3: Go to center
        center = (state.limit_left_pos + state.limit_right_pos) // 2
        error = center - state.position
        
        if abs(error) < 50:  # Close enough to center
            send_zero()  # Zero the position
            state.homing_phase = 0
            state.mode = "idle"
            print(f"Homing: Complete! Center at {center}", flush=True)
            return 0
        
        # Move towards center
        if error > 0:
            return min(state.home_speed, error * 10)
        else:
            return max(-state.home_speed, error * 10)
    
    return 0

# ============ ARDUINO UPLOAD ============
upload_in_progress = False

async def do_upload_async():
    """Async wrapper for Arduino upload"""
    global upload_in_progress
    
    if upload_in_progress:
        await broadcast_message({"type": "UPLOAD_RESULT", "success": False, "message": "Upload already in progress"})
        return
    
    upload_in_progress = True
    await broadcast_message({"type": "UPLOAD_STATUS", "message": "Starting upload..."})
    
    try:
        # Run upload in executor to not block
        loop = asyncio.get_event_loop()
        success, msg = await loop.run_in_executor(executor, upload_arduino_code)
        await broadcast_message({"type": "UPLOAD_RESULT", "success": success, "message": msg})
    except Exception as e:
        await broadcast_message({"type": "UPLOAD_RESULT", "success": False, "message": str(e)})
    finally:
        upload_in_progress = False

# ============ NEAT TRAINING ============
training_process = None

async def start_training():
    """Start NEAT training in background process"""
    global training_process
    
    if training_process and training_process.poll() is None:
        print("Training already running!")
        return
    
    print("Starting NEAT training...", flush=True)
    
    # Clear old checkpoints and status file
    import shutil
    checkpoint_dir = os.path.join(os.path.dirname(__file__), 'neat_swing_up_checkpoints')
    
    if os.path.exists(checkpoint_dir):
        shutil.rmtree(checkpoint_dir)
    if os.path.exists(TRAINING_STATUS_FILE):
        os.remove(TRAINING_STATUS_FILE)
    
    # Start training process (swing-up trainer)
    trainer_path = os.path.join(os.path.dirname(__file__), 'neat_swing_up_trainer.py')
    training_process = subprocess.Popen(
        [sys.executable, trainer_path],
        cwd=os.path.dirname(__file__)
    )
    
    await broadcast_message({"type": "TRAINING_STARTED", "trainer": "swingup"})

async def start_balance_training():
    """Start NEAT balance training in background process"""
    global training_process
    
    if training_process and training_process.poll() is None:
        print("Training already running!")
        return
    
    print("Starting NEAT BALANCE training...", flush=True)
    
    # Clear old checkpoints and status file
    import shutil
    checkpoint_dir = os.path.join(os.path.dirname(__file__), 'neat_balance_checkpoints')
    
    if os.path.exists(checkpoint_dir):
        shutil.rmtree(checkpoint_dir)
    if os.path.exists(TRAINING_STATUS_FILE):
        os.remove(TRAINING_STATUS_FILE)
    
    # Start balance training process
    trainer_path = os.path.join(os.path.dirname(__file__), 'neat_balance_trainer.py')
    training_process = subprocess.Popen(
        [sys.executable, trainer_path],
        cwd=os.path.dirname(__file__)
    )
    
    await broadcast_message({"type": "TRAINING_STARTED", "trainer": "balance"})

async def stop_training():
    """Stop NEAT training"""
    global training_process, last_training_status
    
    if training_process and training_process.poll() is None:
        print("Stopping NEAT training...", flush=True)
        training_process.terminate()
        training_process.wait(timeout=5)
        training_process = None
        
        # Clean up status file
        if os.path.exists(TRAINING_STATUS_FILE):
            os.remove(TRAINING_STATUS_FILE)
        last_training_status = None
        
        # Reload the best genome if it exists
        global neat_network
        load_neat_network()
        
        await broadcast_message({"type": "TRAINING_STOPPED"})
    else:
        print("No training running")

async def delete_best_genome(trainer_type: str = "swing-up"):
    """Delete the best genome file for the specified trainer type"""
    global neat_network, best_genome_info, balance_network, balance_genome_info
    
    # Determine which genome file to delete
    if trainer_type in ["swing-up", "swing_up"]:
        genome_path = os.path.join(os.path.dirname(__file__), 'best_swing_up_genome.pkl')
        label = "swing-up"
    else:
        genome_path = os.path.join(os.path.dirname(__file__), 'best_balance_genome.pkl')
        label = "balance"
    
    if os.path.exists(genome_path):
        os.remove(genome_path)
        
        # Clear the appropriate network and info
        if trainer_type in ["swing-up", "swing_up"]:
            neat_network = None
            best_genome_info = None
        else:
            balance_network = None
            balance_genome_info = None
        
        print(f"Deleted {label} genome: {genome_path}", flush=True)
        await broadcast_message({"type": "DELETE_RESULT", "success": True, "message": f"{label} genome deleted!", "trainer": trainer_type})
    else:
        print(f"No {label} genome to delete", flush=True)
        await broadcast_message({"type": "DELETE_RESULT", "success": False, "message": f"No {label} genome exists"})

async def update_neat_config(cmd: dict):
    """Update NEAT training configuration in JSON file"""
    trainer_type = cmd.get('trainer', 'swing-up')
    pop_size = cmd.get('pop_size', 100)
    max_speed = cmd.get('max_speed', 9000)
    sim_steps = cmd.get('sim_steps', 2000)
    
    # Map tab names to config keys
    key = "swing_up" if trainer_type in ["swing-up", "swing_up"] else "balance"
    
    try:
        # Load existing config
        config_data = load_training_config()
        
        # Update the specific trainer config
        config_data[key] = {
            "pop_size": pop_size,
            "max_speed": max_speed,
            "sim_steps": sim_steps
        }
        
        # Save to JSON
        save_training_config(config_data)
        
        # Update controller's neat_max_speed if this is the swing-up trainer
        if key == "swing_up":
            config.neat_max_speed = max_speed
        
        print(f"NEAT config ({key}) updated: pop={pop_size}, max_speed={max_speed}, steps={sim_steps}", flush=True)
        await broadcast_message({"type": "CONFIG_RESULT", "success": True, "message": f"{key} config updated!"})
        
    except Exception as e:
        print(f"Error updating NEAT config: {e}", flush=True)
        await broadcast_message({"type": "CONFIG_RESULT", "success": False, "message": str(e)})

async def send_neat_config(websocket, trainer_type: str):
    """Send NEAT config for a specific trainer type"""
    config_data = get_neat_config(trainer_type)
    await websocket.send(json.dumps({
        "type": "NEAT_CONFIG_DATA",
        "trainer": trainer_type,
        **config_data
    }))

def upload_arduino_code():
    """Upload Arduino code using arduino-cli (runs in thread)"""
    global serial_port
    
    if SIMULATOR_MODE:
        print("Upload skipped - running in simulator mode", flush=True)
        return True, "Upload skipped (simulator mode)"
    
    print("Starting Arduino upload...", flush=True)
    
    # Close serial connection
    if serial_port and serial_port.is_open:
        serial_port.close()
        state.connected = False
        time.sleep(1)
    
    try:
        # Compile
        print("Compiling...", flush=True)
        result = subprocess.run(
            ["arduino-cli", "compile", "--fqbn", BOARD, SKETCH_PATH],
            capture_output=True, text=True, timeout=120
        )
        if result.returncode != 0:
            print(f"Compile failed: {result.stderr}", flush=True)
            connect_serial()  # Reconnect even on failure
            return False, f"Compile error: {result.stderr}"
        
        # Upload
        print("Uploading...", flush=True)
        result = subprocess.run(
            ["arduino-cli", "upload", "-p", SERIAL_PORT, "--fqbn", BOARD, SKETCH_PATH],
            capture_output=True, text=True, timeout=60
        )
        if result.returncode != 0:
            print(f"Upload failed: {result.stderr}", flush=True)
            connect_serial()  # Reconnect even on failure
            return False, f"Upload error: {result.stderr}"
        
        print("Upload successful!", flush=True)
        time.sleep(2)  # Wait for Arduino to reset
        
        # Reconnect serial
        connect_serial()
        return True, "Upload successful!"
        
    except subprocess.TimeoutExpired:
        connect_serial()
        return False, "Upload timed out"
    except Exception as e:
        connect_serial()
        return False, f"Upload error: {str(e)}"

# ============ WEBSOCKET SERVER ============
async def handle_websocket(websocket):
    """Handle WebSocket client connection"""
    ws_clients.add(websocket)
    print(f"WebSocket client connected. Total: {len(ws_clients)}")
    
    try:
        async for message in websocket:
            await handle_command(message, websocket)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"WebSocket client disconnected. Total: {len(ws_clients)}")

async def handle_command(message: str, websocket=None):
    """Handle command from web UI"""
    try:
        cmd = json.loads(message)
        cmd_type = cmd.get("type", "")
        
        if cmd_type == "MODE":
            new_mode = cmd.get("mode", "idle")
            if new_mode in ["idle", "manual_left", "manual_right", "oscillate", "neat_swing_up_only", "neat_balance_only"]:
                # Reset pendulum to upright in simulator when entering balance-only mode
                if new_mode == "neat_balance_only" and SIMULATOR_MODE and serial_port:
                    serial_port.set_pendulum_upright()
                    print("Simulator: Reset pendulum to upright (180°)")
                
                state.mode = new_mode
                print(f"Mode changed to: {new_mode}")
                if new_mode == "idle":
                    send_stop()
        
        elif cmd_type == "STOP":
            state.mode = "idle"
            send_stop()
        
        elif cmd_type == "ZERO":
            send_zero()
        
        elif cmd_type == "HOME":
            # Start homing sequence
            state.mode = "homing"
            state.homing_phase = 1
            print("Homing: Starting sequence...", flush=True)
        
        elif cmd_type == "CONFIG":
            # Update configuration
            if "manual_speed" in cmd:
                config.manual_speed = int(cmd["manual_speed"])
            if "manual_accel" in cmd:
                config.manual_accel = int(cmd["manual_accel"])
            if "oscillate_speed" in cmd:
                config.oscillate_speed = int(cmd["oscillate_speed"])
            if "oscillate_period" in cmd:
                config.oscillate_period = float(cmd["oscillate_period"])
            if "swingup_kp" in cmd:
                config.swingup_kp = float(cmd["swingup_kp"])
            if "swingup_max_speed" in cmd:
                config.swingup_max_speed = int(cmd["swingup_max_speed"])
            if "balance_kp" in cmd:
                config.balance_kp = float(cmd["balance_kp"])
            if "balance_kd" in cmd:
                config.balance_kd = float(cmd["balance_kd"])
            print(f"Config updated: {cmd}")
        
        elif cmd_type == "UPLOAD":
            # Run upload in background task
            asyncio.create_task(do_upload_async())
        
        elif cmd_type == "START_TRAINING":
            # Start NEAT swing-up training in background
            await start_training()
        
        elif cmd_type == "START_BALANCE_TRAINING":
            # Start NEAT balance training in background
            await start_balance_training()
        
        elif cmd_type == "STOP_TRAINING":
            # Stop NEAT training
            await stop_training()
        
        elif cmd_type == "DELETE_BEST":
            # Delete the best genome for the specified trainer type
            trainer_type = cmd.get('trainer', 'swing-up')
            await delete_best_genome(trainer_type)
        
        elif cmd_type == "NEAT_CONFIG":
            # Update NEAT config
            await update_neat_config(cmd)
        
        elif cmd_type == "GET_NEAT_CONFIG":
            # Get NEAT config for a specific trainer
            trainer_type = cmd.get('trainer', 'swing-up')
            await send_neat_config(websocket, trainer_type)
    
    except json.JSONDecodeError:
        print(f"Invalid command: {message}")
    except Exception as e:
        print(f"Command error: {e}")

async def broadcast_message(msg: dict):
    """Send message to all connected WebSocket clients"""
    if ws_clients:
        data = json.dumps(msg)
        # Copy the set to avoid "Set changed size during iteration"
        clients = list(ws_clients)
        await asyncio.gather(*[client.send(data) for client in clients], return_exceptions=True)

async def broadcast_state():
    """Broadcast current state to all WebSocket clients"""
    if not ws_clients:
        return
    
    data = json.dumps({
        "type": "STATE",
        "angle": round(state.angle, 1),
        "angular_velocity": round(state.angular_velocity, 1),
        "position": state.position,
        "velocity": state.velocity,
        "limit_left": state.limit_left,
        "limit_right": state.limit_right,
        "limit_left_pos": state.limit_left_pos,
        "limit_right_pos": state.limit_right_pos,
        "mode": state.mode,
        "connected": state.connected,
        "best_genome_swing_up": best_genome_info,
        "best_genome_balance": balance_genome_info,
        "neat_config": neat_config_values
    })
    
    # Send to each client, removing dead connections
    dead_clients = set()
    # Copy the set to avoid "Set changed size during iteration"
    clients = list(ws_clients)
    for client in clients:
        try:
            await client.send(data)
        except websockets.exceptions.ConnectionClosed:
            dead_clients.add(client)
        except Exception as e:
            dead_clients.add(client)
    
    # Remove dead clients
    for client in dead_clients:
        ws_clients.discard(client)

# ============ TRAINING STATUS FILE ============
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')
last_training_status = None

async def check_training_status():
    """Read training status from file and broadcast if changed"""
    global last_training_status
    
    if not os.path.exists(TRAINING_STATUS_FILE):
        return
    
    try:
        with open(TRAINING_STATUS_FILE, 'r') as f:
            status = json.load(f)
        
        # Only broadcast if changed
        if status != last_training_status:
            last_training_status = status
            await broadcast_message({"type": "TRAINING", **status})
            print(f"Training: Gen {status.get('generation', '?')}, Best {status.get('best_fitness', '?'):.1f}", flush=True)
    except:
        pass  # File might be being written

# ============ MAIN CONTROL LOOP ============
async def control_loop():
    """Main control loop running at CONTROL_RATE Hz"""
    loop = asyncio.get_event_loop()
    interval = 1.0 / CONTROL_RATE
    broadcast_interval = 0.05  # 20Hz for WebSocket updates (not 100Hz)
    last_broadcast = 0
    last_training_check = 0
    
    # Give WebSocket server time to start accepting connections
    print("Waiting for WebSocket to be ready...", flush=True)
    await asyncio.sleep(0.5)
    print("Control loop starting...", flush=True)
    
    last_printed_velocity = 0
    
    while True:
        start = time.perf_counter()
        
        # Read sensor data in thread pool (non-blocking)
        await loop.run_in_executor(executor, read_serial)
        
        # Compute velocity
        velocity = compute_velocity()
        
        # Debug: print when velocity changes significantly
        if abs(velocity - last_printed_velocity) > 100:
            print(f"Mode: {state.mode}, Velocity: {velocity}", flush=True)
            last_printed_velocity = velocity
        
        # Update limit positions when switches are hit
        if state.limit_left:
            state.limit_left_pos = state.position
        if state.limit_right:
            state.limit_right_pos = state.position
        
        # Send velocity in thread pool (non-blocking)
        await loop.run_in_executor(executor, send_velocity, velocity)
        
        # Broadcast state to web clients at lower rate
        now = time.perf_counter()
        if now - last_broadcast >= broadcast_interval:
            last_broadcast = now
            try:
                await broadcast_state()
                # Check training status file (every 0.5s)
                if now - last_training_check >= 0.5:
                    last_training_check = now
                    await check_training_status()
            except Exception as e:
                print(f"Broadcast error: {e}", flush=True)
        
        # Sleep to maintain loop rate
        elapsed = time.perf_counter() - start
        sleep_time = max(0.01, interval - elapsed)  # Minimum 10ms sleep
        await asyncio.sleep(sleep_time)

async def main():
    """Main entry point"""
    print("=" * 50)
    print("Inverted Pendulum - Velocity Streaming Controller")
    if SIMULATOR_MODE:
        print(">>> SIMULATOR MODE - No hardware required <<<")
    print("=" * 50, flush=True)
    
    # Load NEAT networks and config
    load_neat_network()
    load_balance_network()
    get_neat_config()
    
    # Connect to Arduino or simulator
    if not connect_serial():
        print("Warning: Could not connect to Arduino", flush=True)
    
    # Start WebSocket server FIRST
    print(f"Starting WebSocket server on ws://{WS_HOST}:{WS_PORT}...", flush=True)
    server = await websockets.serve(handle_websocket, WS_HOST, WS_PORT)
    print("WebSocket server started!", flush=True)
    
    print(f"Starting control loop at {CONTROL_RATE} Hz", flush=True)
    print("Ready! Open web interface to control.", flush=True)
    
    try:
        # Run control loop (WebSocket server runs in background automatically)
        await control_loop()
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        server.close()
        await server.wait_closed()
        if serial_port:
            serial_port.close()

if __name__ == "__main__":
    asyncio.run(main())

