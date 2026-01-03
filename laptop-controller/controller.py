"""
Inverted Pendulum - Acceleration Control Controller

All control logic runs here in Python.
Arduino is a simple sensor/actuator interface.

Communication:
- Serial to Arduino: Velocity commands "V1234\n" (acceleration integrated to velocity for hardware)
- Serial from Arduino: "A:180.5,P:1000,LL:0,LR:1,V:1234\n"
- WebSocket to Web UI: JSON state updates

Controller uses acceleration-only control internally. For real hardware,
acceleration is integrated to velocity before sending to Arduino.

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

# Import simulator (always available, even if not used)
try:
    from simulator import PendulumSimulator
except ImportError:
    PendulumSimulator = None

# EvoTorch model
evotorch_model = None

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
    mode: str = "idle"             # idle, manual_left, manual_right, manual_left_accel, manual_right_accel, oscillate, evotorch_balance, homing
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
    

# Global state
state = PendulumState()
config = ControlConfig()
ws_clients: set = set()
serial_port: Optional[serial.Serial] = None
executor = ThreadPoolExecutor(max_workers=1)  # For serial I/O

# For angular velocity calculation
last_angle = 180.0
last_angle_time = time.perf_counter()

# For acceleration delta tracking (for EvoTorch mode)
last_acceleration = 0.0

TRAINING_CONFIG_FILE = os.path.join(os.path.dirname(__file__), 'neat_training_config.json')

def load_training_config():
    """Load training config from JSON file"""
    try:
        if os.path.exists(TRAINING_CONFIG_FILE):
            with open(TRAINING_CONFIG_FILE, 'r') as f:
                return json.load(f)
    except:
        pass
    # Default config (EvoTorch only)
    return {
        "evotorch": {"population_size": 50, "max_speed": 9000, "sim_steps": 5000, "generations": 0}
    }

def save_training_config(config: dict):
    """Save training config to JSON file"""
    try:
        with open(TRAINING_CONFIG_FILE, 'w') as f:
            json.dump(config, f, indent=2)
            f.flush()
            os.fsync(f.fileno())
    except Exception as e:
        print(f"Error saving config: {e}")

def load_evotorch_model():
    """Load the EvoTorch model if available"""
    global evotorch_model
    
    model_path = os.path.join(os.path.dirname(__file__), 'evotorch_balance_model.pkl')
    
    if not os.path.exists(model_path):
        print(f"EvoTorch: No trained model found at {model_path}", flush=True)
        evotorch_model = None
        return False
    
    try:
        import pickle
        import torch
        import torch.nn as nn
        
        # Import BalancePolicy class so pickle can deserialize it
        # The model was saved from evotorch_balance_trainer, so we need to make sure
        # pickle can find the class in the right namespace
        try:
            from evotorch_balance_trainer import BalancePolicy
            print("EvoTorch: Imported BalancePolicy from trainer module", flush=True)
            
            # Make sure pickle can find it - register it in sys.modules if needed
            import sys
            # If the model was saved from __main__, we need to make BalancePolicy available there
            if '__main__' in sys.modules:
                main_module = sys.modules['__main__']
                if not hasattr(main_module, 'BalancePolicy'):
                    main_module.BalancePolicy = BalancePolicy
                    print("EvoTorch: Registered BalancePolicy in __main__ module", flush=True)
        except ImportError as import_err:
            print(f"EvoTorch: Could not import BalancePolicy, defining inline: {import_err}", flush=True)
            # Define BalancePolicy class if import fails
            class BalancePolicy(nn.Module):
                """Neural network policy for balance control (2 hidden layers, 10 nodes each)"""
                def __init__(self, input_size=5, hidden_size=10, output_size=1):
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
            
            # Register in __main__ for pickle
            import sys
            if '__main__' in sys.modules:
                sys.modules['__main__'].BalancePolicy = BalancePolicy
        
        print(f"EvoTorch: Loading model from {model_path}...", flush=True)
        with open(model_path, 'rb') as f:
            evotorch_model = pickle.load(f)
        
        if hasattr(evotorch_model, 'eval'):
            evotorch_model.eval()
        
        # Test the model with a dummy input to make sure it works
        test_input = torch.tensor([0.0, 1.0, 0.0, 0.0, 0.0], dtype=torch.float32)
        with torch.no_grad():
            _ = evotorch_model(test_input)
        
        print(f"EvoTorch: Successfully loaded trained model from {model_path}", flush=True)
        return True
    except Exception as e:
        print(f"EvoTorch: Failed to load model: {e}", flush=True)
        import traceback
        traceback.print_exc()
        evotorch_model = None
        return False


# ============ SERIAL COMMUNICATION ============
def connect_serial():
    global serial_port
    
    if SIMULATOR_MODE:
        # Use physics simulator instead of real serial
        if PendulumSimulator is None:
            raise ImportError("Simulator module not found. Cannot use --sim mode.")
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
    """Send velocity command to Arduino or simulator"""
    if SIMULATOR_MODE and isinstance(serial_port, PendulumSimulator):
        # Simulator supports velocity directly
        serial_port.set_target_velocity(float(velocity))
    elif serial_port and serial_port.is_open:
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

def send_acceleration(acceleration: int, is_delta: bool = False):
    """Send acceleration command (for simulator) or convert to velocity (for Arduino)
    
    Args:
        acceleration: Acceleration value (absolute or delta depending on is_delta)
        is_delta: If True, acceleration is a delta (change) from current, otherwise absolute
    """
    global last_acceleration
    
    # Check limits and stop acceleration if at limit
    if state.limit_left and acceleration < 0:
        acceleration = 0
    if state.limit_right and acceleration > 0:
        acceleration = 0
    
    if SIMULATOR_MODE and isinstance(serial_port, PendulumSimulator):
        # Simulator supports acceleration directly
        if is_delta:
            # For delta mode: add to current acceleration
            # Get current acceleration from simulator state if possible, otherwise track it
            current_accel = last_acceleration
            target_accel = current_accel + float(acceleration)
            last_acceleration = target_accel
            serial_port.set_target_acceleration(target_accel)
        else:
            # Absolute mode: set directly (matches training)
            last_acceleration = float(acceleration)
            serial_port.set_target_acceleration(float(acceleration))
    elif serial_port and serial_port.is_open:
        # For real hardware, convert acceleration to velocity change
        # Simple integration: v = v_prev + a * dt
        CONTROL_DT = 0.02  # Control loop period (50Hz)
        velocity_change = acceleration * CONTROL_DT
        target_velocity = state.velocity + velocity_change
        # Clamp to max speed
        max_speed = config.manual_speed * 10  # Allow higher speeds for acceleration mode
        target_velocity = max(-max_speed, min(max_speed, target_velocity))
        send_velocity(int(target_velocity))

def send_stop():
    """Send stop command (immediately stop cart by setting velocity and acceleration to zero)"""
    if SIMULATOR_MODE and isinstance(serial_port, PendulumSimulator):
        # For simulator: immediately stop (set both velocity and acceleration to zero)
        serial_port.stop()
    elif serial_port and serial_port.is_open:
        # For real hardware: send stop command via serial
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
def compute_acceleration() -> int:
    """Compute desired acceleration based on current mode (acceleration-only control)"""
    
    if state.mode == "idle":
        return 0
    
    elif state.mode == "manual_left":
        if state.limit_left:  # Left limit blocks moving left
            return 0
        # Always use acceleration mode (simplified to acceleration-only)
        return -config.manual_accel  # Negative = left
    
    elif state.mode == "manual_right":
        if state.limit_right:  # Right limit blocks moving right
            return 0
        # Always use acceleration mode (simplified to acceleration-only)
        return config.manual_accel  # Positive = right
    
    elif state.mode == "manual_left_accel":
        if state.limit_left:  # Left limit blocks moving left
            return 0
        return -config.manual_accel  # Negative = left (acceleration)
    
    elif state.mode == "manual_right_accel":
        if state.limit_right:  # Right limit blocks moving right
            return 0
        return config.manual_accel  # Positive = right (acceleration)
    
    elif state.mode == "oscillate":
        return compute_oscillate()
    
    elif state.mode == "evotorch_balance":
        return compute_evotorch_balance()
    
    elif state.mode == "homing":
        return compute_homing()
    
    return 0

def compute_oscillate() -> int:
    """Simple sinusoidal oscillation (returns acceleration)"""
    t = time.perf_counter()
    phase = (t % config.oscillate_period) / config.oscillate_period * 2 * math.pi
    # Convert velocity profile to acceleration by taking derivative
    # v = speed * sin(phase), a = dv/dt = speed * cos(phase) * d(phase)/dt
    phase_rate = 2 * math.pi / config.oscillate_period
    acceleration = int(config.oscillate_speed * math.cos(phase) * phase_rate)
    
    # Respect limits
    if state.limit_left and acceleration < 0:
        return 0
    if state.limit_right and acceleration > 0:
        return 0
    
    return acceleration

def compute_evotorch_balance() -> int:
    """Use EvoTorch model to control the pendulum (acceleration-only control)
    
    This matches the training evaluation exactly:
    - State normalization: sin(θ), cos(θ), angular_vel/1000, cart_pos/(rail_length/2), cart_vel/9000
    - Max acceleration: 500000 (matches training motor_accel)
    - Rail length: 7500 steps (matches training)
    """
    global evotorch_model
    
    # Try to reload if None (in case model was saved after controller started)
    if evotorch_model is None:
        print("EvoTorch model not loaded, attempting to reload...", flush=True)
        if load_evotorch_model():
            print("EvoTorch model reloaded successfully!", flush=True)
        else:
            print("EvoTorch model not loaded! Going to idle.", flush=True)
            state.mode = "idle"
            return 0
    
    # Get rail limits for normalization - match training exactly
    # Training uses: rail_length = limit_right_pos - limit_left_pos, then rail_length / 2
    # Training rail_length_steps = 7500
    rail_length = state.limit_right_pos - state.limit_left_pos
    if rail_length <= 0:
        # Fallback if limits not set yet
        rail_length = 7500
    rail_half = rail_length / 2
    
    # Training constants
    MAX_SPEED = 9000  # Matches evotorch_balance_trainer.py
    MAX_ACCEL = 500000  # Matches training motor_accel
    
    # Prepare inputs using sin(θ) and cos(θ) to match trainer exactly (5 inputs)
    import math
    import torch
    angle_rad = math.radians(state.angle)
    state_vec = torch.tensor([
        math.sin(angle_rad),  # sin(θ)
        math.cos(angle_rad),  # cos(θ)
        state.angular_velocity / 1000.0,  # Angular velocity (normalized) - matches training
        state.position / rail_half if rail_half > 0 else 0,  # Cart position (normalized) - matches training
        state.velocity / MAX_SPEED  # Cart velocity (normalized) - matches training MAX_SPEED = 9000
    ], dtype=torch.float32)
    
    # Get action from model
    with torch.no_grad():
        action = evotorch_model(state_vec).item()
    
    # Clip to [-1, 1] range (tanh already outputs in this range, but ensure it)
    action = max(-1.0, min(1.0, action))
    
    # Convert to acceleration - use training's max acceleration
    acceleration = action * MAX_ACCEL
    
    
    # Respect limits
    if state.limit_left and acceleration < 0:
        acceleration = 0
    if state.limit_right and acceleration > 0:
        acceleration = 0
    
    # Return acceleration (will be handled by send_acceleration)
    return int(acceleration)

def compute_homing() -> int:
    """
    Homing sequence (returns acceleration):
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
        # Use acceleration for homing
        return config.manual_accel // 2  # Positive = right (reduced acceleration)
    
    elif state.homing_phase == 2:
        # Phase 2: Going left to find left limit
        if state.limit_left:
            state.limit_left_pos = state.position
            state.homing_phase = 3
            print(f"Homing: Found left limit at {state.limit_left_pos}", flush=True)
            return 0
        return -config.manual_accel // 2  # Negative = left (reduced acceleration)
    
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
        
        # Move towards center with proportional acceleration
        max_homing_accel = config.manual_accel // 2
        if error > 0:
            return min(max_homing_accel, error * 100)
        else:
            return max(-max_homing_accel, error * 100)
    
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

# ============ TRAINING ============
training_process = None
TRAINING_STATUS_FILE = os.path.join(os.path.dirname(__file__), 'training_status.json')

async def start_evotorch_training():
    """Start EvoTorch training in background process"""
    global training_process
    
    if training_process and training_process.poll() is None:
        print("Training already running!")
        return
    
    print("Starting EvoTorch training...", flush=True)
    
    # Clear old status file
    if os.path.exists(TRAINING_STATUS_FILE):
        os.remove(TRAINING_STATUS_FILE)
    
    # Start evotorch training process
    trainer_path = os.path.join(os.path.dirname(__file__), 'evotorch_balance_trainer.py')
    training_process = subprocess.Popen(
        [sys.executable, trainer_path],
        cwd=os.path.dirname(__file__)
    )
    
    await broadcast_message({"type": "TRAINING_STARTED", "trainer": "evotorch"})

async def stop_training():
    """Stop training (works for evotorch)"""
    global training_process, last_training_status
    
    if training_process and training_process.poll() is None:
        print("Stopping training...", flush=True)
        
        # Create stop flag for trainers
        stop_flag = os.path.join(os.path.dirname(__file__), 'stop_training.flag')
        with open(stop_flag, 'w') as f:
            f.write('stop')
        
        # Terminate process
        training_process.terminate()
        try:
            training_process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            training_process.kill()
        training_process = None
        
        # Clean up files
        if os.path.exists(TRAINING_STATUS_FILE):
            os.remove(TRAINING_STATUS_FILE)
        if os.path.exists(stop_flag):
            os.remove(stop_flag)
        last_training_status = None
        
        # Reload EvoTorch model if it exists
        global evotorch_model
        load_evotorch_model()
        
        await broadcast_message({"type": "TRAINING_STOPPED"})
    else:
        print("No training running")

async def update_evotorch_config(cmd: dict):
    """Update EvoTorch training configuration in JSON file"""
    population_size = cmd.get('population_size', 50)
    max_speed = cmd.get('max_speed', 9000)
    sim_steps = cmd.get('sim_steps', 5000)
    generations = cmd.get('generations', 0)  # 0 means indefinite
    
    try:
        # Load existing config
        config_data = load_training_config()
        
        # Update evotorch config
        config_data['evotorch'] = {
            "population_size": population_size,
            "max_speed": max_speed,
            "sim_steps": sim_steps,
            "generations": generations
        }
        
        # Save to file
        save_training_config(config_data)
        
        print(f"EvoTorch config updated: population={population_size}, max_speed={max_speed}, sim_steps={sim_steps}, generations={generations}", flush=True)
        await broadcast_message({"type": "CONFIG_RESULT", "success": True, "message": "EvoTorch config updated!"})
    except Exception as e:
        print(f"Error updating EvoTorch config: {e}", flush=True)
        await broadcast_message({"type": "CONFIG_RESULT", "success": False, "message": str(e)})

async def send_evotorch_generations(websocket):
    """Send generation history to client"""
    history_file = os.path.join(os.path.dirname(__file__), 'evotorch_generation_history.json')
    
    try:
        if os.path.exists(history_file):
            with open(history_file, 'r') as f:
                history = json.load(f)
        else:
            history = {}
        
        await websocket.send(json.dumps({
            "type": "EVOTORCH_GENERATIONS",
            "generations": history
        }))
    except Exception as e:
        print(f"Error sending generation history: {e}", flush=True)
        await websocket.send(json.dumps({
            "type": "EVOTORCH_GENERATIONS",
            "generations": {},
            "error": str(e)
        }))

async def send_evotorch_population_records(websocket, generation: int):
    """Send population recordings for a specific generation and automatically send all recordings"""
    checkpoint_dir = os.path.join(os.path.dirname(__file__), 'evotorch_checkpoints')
    records_dir = os.path.join(checkpoint_dir, f'generation_{generation}_records')
    
    try:
        if not os.path.exists(records_dir):
            await websocket.send(json.dumps({
                "type": "EVOTORCH_POPULATION_RECORDS",
                "generation": generation,
                "records": {},
                "error": f"No records found for generation {generation}"
            }))
            return
        
        # Load all solution recordings
        records = {}
        solution_files = []
        for filename in os.listdir(records_dir):
            if filename.startswith('solution_') and filename.endswith('.json'):
                solution_id = int(filename.replace('solution_', '').replace('.json', ''))
                record_path = os.path.join(records_dir, filename)
                solution_files.append((solution_id, record_path))
                try:
                    with open(record_path, 'r') as f:
                        record_data = json.load(f)
                    records[solution_id] = {
                        'solution_id': solution_id,
                        'fitness': record_data.get('fitness', 0),
                        'trajectory_length': len(record_data.get('trajectory', [])),
                        'generation': generation
                    }
                except Exception as e:
                    print(f"Error loading record {filename}: {e}", flush=True)
        
        # Send the records list first
        await websocket.send(json.dumps({
            "type": "EVOTORCH_POPULATION_RECORDS",
            "generation": generation,
            "records": records
        }))
        
        # Automatically send all recordings
        for solution_id, record_path in sorted(solution_files):
            try:
                with open(record_path, 'r') as f:
                    recording_data = json.load(f)
                await websocket.send(json.dumps({
                    "type": "EVOTORCH_SOLUTION_RECORDING",
                    "generation": generation,
                    "solution_id": solution_id,
                    "recording": recording_data
                }))
            except Exception as e:
                print(f"Error sending recording for solution {solution_id}: {e}", flush=True)
        
    except Exception as e:
        print(f"Error sending population records: {e}", flush=True)
        import traceback
        traceback.print_exc()
        await websocket.send(json.dumps({
            "type": "EVOTORCH_POPULATION_RECORDS",
            "generation": generation,
            "records": {},
            "error": str(e)
        }))

async def send_evotorch_solution_recording(websocket, generation: int, solution_id: int):
    """Send a specific solution's recording for replay"""
    checkpoint_dir = os.path.join(os.path.dirname(__file__), 'evotorch_checkpoints')
    records_dir = os.path.join(checkpoint_dir, f'generation_{generation}_records')
    record_file = os.path.join(records_dir, f'solution_{solution_id}.json')
    
    try:
        if not os.path.exists(record_file):
            await websocket.send(json.dumps({
                "type": "EVOTORCH_SOLUTION_RECORDING",
                "generation": generation,
                "solution_id": solution_id,
                "recording": None,
                "error": f"Recording not found for generation {generation}, solution {solution_id}"
            }))
            return
        
        with open(record_file, 'r') as f:
            recording_data = json.load(f)
        
        await websocket.send(json.dumps({
            "type": "EVOTORCH_SOLUTION_RECORDING",
            "generation": generation,
            "solution_id": solution_id,
            "recording": recording_data
        }))
    except Exception as e:
        print(f"Error sending solution recording: {e}", flush=True)
        import traceback
        traceback.print_exc()
        await websocket.send(json.dumps({
            "type": "EVOTORCH_SOLUTION_RECORDING",
            "generation": generation,
            "solution_id": solution_id,
            "recording": None,
            "error": str(e)
        }))

async def preview_evotorch_generation(websocket, generation: int):
    """Load and preview a specific generation's model"""
    global evotorch_model
    
    history_file = os.path.join(os.path.dirname(__file__), 'evotorch_generation_history.json')
    checkpoint_dir = os.path.join(os.path.dirname(__file__), 'evotorch_checkpoints')
    
    try:
        # Load generation history
        if not os.path.exists(history_file):
            await websocket.send(json.dumps({
                "type": "PREVIEW_RESULT",
                "success": False,
                "message": "No generation history found"
            }))
            return
        
        with open(history_file, 'r') as f:
            history = json.load(f)
        
        gen_key = str(generation)
        if gen_key not in history:
            await websocket.send(json.dumps({
                "type": "PREVIEW_RESULT",
                "success": False,
                "message": f"Generation {generation} not found"
            }))
            return
        
        gen_info = history[gen_key]
        checkpoint_path = gen_info.get('checkpoint')
        
        if not checkpoint_path or not os.path.exists(checkpoint_path):
            # Try to construct path
            checkpoint_path = os.path.join(checkpoint_dir, f'generation_{generation}.pkl')
            if not os.path.exists(checkpoint_path):
                await websocket.send(json.dumps({
                    "type": "PREVIEW_RESULT",
                    "success": False,
                    "message": f"Checkpoint file not found for generation {generation}"
                }))
                return
        
        # Import BalancePolicy class
        try:
            from evotorch_balance_trainer import BalancePolicy
            import sys
            if '__main__' in sys.modules:
                sys.modules['__main__'].BalancePolicy = BalancePolicy
        except ImportError:
            # Define inline if import fails
            import torch.nn as nn
            import torch
            class BalancePolicy(nn.Module):
                def __init__(self, input_size=5, hidden_size=10, output_size=1):
                    super(BalancePolicy, self).__init__()
                    self.fc1 = nn.Linear(input_size, hidden_size)
                    self.fc2 = nn.Linear(hidden_size, hidden_size)
                    self.fc3 = nn.Linear(hidden_size, output_size)
                    self.activation = nn.Tanh()
                
                def forward(self, x):
                    x = self.activation(self.fc1(x))
                    x = self.activation(self.fc2(x))
                    x = torch.tanh(self.fc3(x))
                    return x
        
        # Load the model
        import pickle
        with open(checkpoint_path, 'rb') as f:
            preview_model = pickle.load(f)
        
        preview_model.eval()
        
        # Temporarily replace the current model for preview
        evotorch_model = preview_model
        
        # Switch to evotorch_balance mode for preview
        state.mode = "evotorch_balance"
        
        await websocket.send(json.dumps({
            "type": "PREVIEW_RESULT",
            "success": True,
            "generation": generation,
            "fitness": gen_info.get('fitness'),
            "best_fitness": gen_info.get('best_fitness'),
            "message": f"Previewing generation {generation}. Click 'Stop' to exit preview."
        }))
        
        print(f"Previewing generation {generation} (fitness: {gen_info.get('fitness', 0):.1f})", flush=True)
        
    except Exception as e:
        print(f"Error previewing generation {generation}: {e}", flush=True)
        import traceback
        traceback.print_exc()
        await websocket.send(json.dumps({
            "type": "PREVIEW_RESULT",
            "success": False,
            "message": f"Error: {str(e)}"
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
            if new_mode in ["idle", "manual_left", "manual_right", "manual_left_accel", "manual_right_accel", "oscillate", "evotorch_balance"]:
                # Reset pendulum to upright in simulator when entering balance modes
                if new_mode in ["evotorch_balance"] and SIMULATOR_MODE and serial_port:
                    serial_port.set_pendulum_upright()
                    print(f"Simulator: Reset pendulum to upright (180°) for {new_mode}")
                
                state.mode = new_mode
                print(f"Mode changed to: {new_mode}")
                # Note: idle mode allows movement to continue, only STOP command stops movement
        
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
        
        elif cmd_type == "START_EVOTORCH_TRAINING":
            # Start EvoTorch training in background
            await start_evotorch_training()
        
        elif cmd_type == "STOP_TRAINING":
            # Stop training
            await stop_training()
        
        elif cmd_type == "EVOTORCH_CONFIG":
            # Update evotorch config
            await update_evotorch_config(cmd)
        
        elif cmd_type == "GET_EVOTORCH_GENERATIONS":
            # Send generation history
            await send_evotorch_generations(websocket)
        
        elif cmd_type == "PREVIEW_EVOTORCH_GENERATION":
            # Load and preview a specific generation
            generation = cmd.get('generation')
            await preview_evotorch_generation(websocket, generation)
        
        elif cmd_type == "GET_EVOTORCH_POPULATION_RECORDS":
            # Get population recordings for a generation
            generation = cmd.get('generation')
            await send_evotorch_population_records(websocket, generation)
        
        elif cmd_type == "GET_EVOTORCH_SOLUTION_RECORDING":
            # Get a specific solution's recording
            generation = cmd.get('generation')
            solution_id = cmd.get('solution_id')
            await send_evotorch_solution_recording(websocket, generation, solution_id)
        
    
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
        "config": {
            "manual_speed": config.manual_speed,
            "manual_accel": config.manual_accel,
            "oscillate_speed": config.oscillate_speed,
            "oscillate_period": config.oscillate_period
        }
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
    global last_training_status, training_process
    
    # Check if training process finished
    if training_process and training_process.poll() is not None:
        # Process finished - reload models in case new ones were saved
        print("Training process finished, reloading models...", flush=True)
        load_evotorch_model()
        training_process = None
        await broadcast_message({"type": "TRAINING_STOPPED"})
    
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
        
        # Update control rate if mode changed (for EvoTorch matching)
        if state.mode == "evotorch_balance":
            control_rate = 50  # Match training EVAL_DT = 0.02 (50Hz)
        else:
            control_rate = CONTROL_RATE
        interval = 1.0 / control_rate
        
        # Read sensor data in thread pool (non-blocking)
        await loop.run_in_executor(executor, read_serial)
        
        # Compute acceleration (acceleration-only control)
        acceleration = compute_acceleration()
        
        # Send acceleration command in thread pool (non-blocking)
        # Send absolute acceleration (matches training: each step sets absolute value)
        # Training does: sim.set_target_acceleration(absolute_acceleration), not delta
        await loop.run_in_executor(executor, send_acceleration, int(acceleration), False)
        
        # Debug: print when acceleration changes significantly
        if abs(acceleration - last_printed_velocity) > 1000:
            print(f"Mode: {state.mode}, Acceleration: {acceleration}", flush=True)
            last_printed_velocity = acceleration
        
        # Update limit positions when switches are hit
        if state.limit_left:
            state.limit_left_pos = state.position
        if state.limit_right:
            state.limit_right_pos = state.position
        
        # Send command in thread pool (non-blocking) - already handled above
        
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
    print("Inverted Pendulum - Acceleration Control Controller")
    if SIMULATOR_MODE:
        print(">>> SIMULATOR MODE - No hardware required <<<")
    print("=" * 50, flush=True)
    
    # Load EvoTorch model
    load_evotorch_model()
    
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

