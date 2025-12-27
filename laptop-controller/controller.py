"""
Inverted Pendulum - Velocity Streaming Controller

All control logic runs here in Python.
Arduino is a simple sensor/actuator interface.

Communication:
- Serial to Arduino: Velocity commands "V1234\n"
- Serial from Arduino: "A:180.5,P:1000,LL:0,LR:1,V:1234\n"
- WebSocket to Web UI: JSON state updates
"""

import serial
import asyncio
import websockets
import json
import time
import math
import subprocess
import os
import threading
from dataclasses import dataclass, asdict
from typing import Optional
from collections import deque
from concurrent.futures import ThreadPoolExecutor

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
    mode: str = "idle"             # idle, manual_left, manual_right, oscillate, swing_up, balance, homing
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
    
    # Swing-up mode
    swingup_kp: float = 8000.0     # Proportional gain for energy pumping
    swingup_max_speed: int = 10000
    
    # Balance mode (PD control)
    balance_kp: float = 500.0
    balance_kd: float = 50.0
    balance_threshold: float = 30.0  # degrees from vertical to switch to balance

# Global state
state = PendulumState()
config = ControlConfig()
ws_clients: set = set()
serial_port: Optional[serial.Serial] = None
executor = ThreadPoolExecutor(max_workers=1)  # For serial I/O

# For angular velocity calculation
last_angle = 180.0
last_angle_time = time.perf_counter()

# ============ SERIAL COMMUNICATION ============
def connect_serial():
    global serial_port
    try:
        serial_port = serial.Serial(SERIAL_PORT, BAUD, timeout=0.01)
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
        cmd = f"V{velocity}\n"
        serial_port.write(cmd.encode())
        # Debug: print when velocity changes significantly
        if abs(velocity - last_debug_velocity[0]) > 100:
            print(f"Sending velocity: {velocity}", flush=True)
            last_debug_velocity[0] = velocity

def send_stop():
    """Send stop command"""
    if serial_port and serial_port.is_open:
        serial_port.write(b"S\n")

def send_zero():
    """Zero the encoder and position"""
    if serial_port and serial_port.is_open:
        # Adjust limit positions relative to new zero
        current_pos = state.position
        if state.limit_left_pos != 0:
            state.limit_left_pos -= current_pos
        if state.limit_right_pos != 0:
            state.limit_right_pos -= current_pos
        serial_port.write(b"Z\n")
        print(f"Zeroed. Limits now: L={state.limit_left_pos}, R={state.limit_right_pos}", flush=True)

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
    
    elif state.mode == "swing_up":
        return compute_swing_up()
    
    elif state.mode == "balance":
        return compute_balance()
    
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

def compute_swing_up() -> int:
    """
    Energy-based swing-up control.
    Move in the direction that adds energy when pendulum is moving.
    
    Key insight: To add energy, move cart in same direction as pendulum tip.
    When pendulum swings right (positive angular velocity at bottom), 
    moving cart right adds energy.
    
    cos(angle) gives the horizontal component:
    - At bottom (180°): cos(180°) = -1
    - At top (0°/360°): cos(0°) = 1
    
    Formula: velocity = Kp * cos(angle) * angular_velocity
    """
    # Convert to radians, with 0 = up
    angle_from_up = math.radians(state.angle)
    
    # Energy pumping: move with the pendulum when it's near bottom
    # cos(angle) is negative at bottom, positive at top
    pump_direction = math.cos(angle_from_up) * state.angular_velocity
    
    velocity = int(config.swingup_kp * pump_direction)
    
    # Clamp to max speed
    velocity = max(-config.swingup_max_speed, min(config.swingup_max_speed, velocity))
    
    # Respect limits
    if state.limit_left and velocity < 0:
        velocity = 0
    if state.limit_right and velocity > 0:
        velocity = 0
    
    # Check if we should switch to balance mode
    angle_from_vertical = min(state.angle, 360 - state.angle)
    if angle_from_vertical < config.balance_threshold:
        state.mode = "balance"
        print("Switching to BALANCE mode!")
    
    return velocity

def compute_balance() -> int:
    """
    PD controller to balance pendulum at top.
    
    Target: angle = 0 (or 360)
    Error: how far from vertical
    """
    # Calculate error from vertical (0 degrees)
    # Normalize to [-180, 180] range
    error = state.angle
    if error > 180:
        error = error - 360
    
    # PD control
    velocity = int(config.balance_kp * error + config.balance_kd * state.angular_velocity)
    
    # Clamp
    max_balance_speed = 20000
    velocity = max(-max_balance_speed, min(max_balance_speed, velocity))
    
    # Respect limits
    if state.limit_left and velocity < 0:
        velocity = 0
    if state.limit_right and velocity > 0:
        velocity = 0
    
    # If we've fallen too far, go back to swing-up
    angle_from_vertical = min(state.angle, 360 - state.angle)
    if angle_from_vertical > 45:  # Fallen past 45 degrees
        state.mode = "swing_up"
        print("Fell! Back to SWING_UP mode")
    
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

def upload_arduino_code():
    """Upload Arduino code using arduino-cli (runs in thread)"""
    global serial_port
    
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
            await handle_command(message)
    except websockets.exceptions.ConnectionClosed:
        pass
    finally:
        ws_clients.discard(websocket)
        print(f"WebSocket client disconnected. Total: {len(ws_clients)}")

async def handle_command(message: str):
    """Handle command from web UI"""
    try:
        cmd = json.loads(message)
        cmd_type = cmd.get("type", "")
        
        if cmd_type == "MODE":
            new_mode = cmd.get("mode", "idle")
            if new_mode in ["idle", "manual_left", "manual_right", "oscillate", "swing_up", "balance"]:
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
    
    except json.JSONDecodeError:
        print(f"Invalid command: {message}")
    except Exception as e:
        print(f"Command error: {e}")

async def broadcast_message(msg: dict):
    """Send message to all connected WebSocket clients"""
    if ws_clients:
        data = json.dumps(msg)
        await asyncio.gather(*[client.send(data) for client in ws_clients], return_exceptions=True)

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
        "connected": state.connected
    })
    
    # Send to each client, removing dead connections
    dead_clients = set()
    for client in ws_clients:
        try:
            await client.send(data)
        except websockets.exceptions.ConnectionClosed:
            dead_clients.add(client)
        except Exception as e:
            dead_clients.add(client)
    
    # Remove dead clients
    for client in dead_clients:
        ws_clients.discard(client)

# ============ MAIN CONTROL LOOP ============
async def control_loop():
    """Main control loop running at CONTROL_RATE Hz"""
    loop = asyncio.get_event_loop()
    interval = 1.0 / CONTROL_RATE
    broadcast_interval = 0.05  # 20Hz for WebSocket updates (not 100Hz)
    last_broadcast = 0
    
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
    print("=" * 50, flush=True)
    
    # Connect to Arduino
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

