"""
Inverted Pendulum Hardware Simulator

Simulates the physical system:
- Cart on a linear rail with stepper motor
- Pendulum attached to cart with rotary encoder
- Limit switches at both ends

Physics model:
- Cart: driven by velocity commands, has inertia
- Pendulum: gravity, damping, coupling to cart acceleration
"""

import math
import time
import threading
from dataclasses import dataclass
from typing import Optional
import io


@dataclass
class SimulatorConfig:
    """Configuration for the simulator"""
    # Rail/Cart parameters
    rail_length_steps: int = 7500        # Total rail length in steps
    cart_mass: float = 0.5               # kg
    motor_accel: float = 500000           # steps/s² acceleration limit
    
    # Pendulum parameters  
    pendulum_length: float = 0.3         # meters
    pendulum_mass: float = 0.1           # kg
    gravity: float = 9.81                # m/s²
    damping: float = 0.2                 # Angular damping coefficient (higher = settles faster)
    
    # Encoder
    encoder_ppr: int = 20                # Pulses per revolution
    encoder_multiplier: int = 4          # Quadrature multiplier
    
    # Timing
    physics_dt: float = 0.001            # Physics timestep (1ms)
    data_interval: float = 0.02          # Sensor output interval (20ms = 50Hz)


class PendulumSimulator:
    """
    Simulates the inverted pendulum hardware.
    Implements a serial-like interface for drop-in replacement.
    """
    
    def __init__(self, config: Optional[SimulatorConfig] = None):
        self.config = config or SimulatorConfig()
        
        # Cart state (in steps)
        self.cart_position: float = 0.0          # Current position (steps)
        self.cart_velocity: float = 0.0          # Current velocity (steps/s)
        self.target_velocity: float = 0.0        # Commanded velocity (steps/s)
        
        # Pendulum state (0° = DOWN/stable, 180° = UP/inverted)
        # Start at 90° (horizontal) so it falls to 0° (down)
        self.pendulum_angle: float = math.pi / 2  # Radians (90° = horizontal)
        self.pendulum_velocity: float = 0.0       # rad/s
        
        # Limit switch positions (in steps from center)
        self.limit_left_pos: float = -self.config.rail_length_steps / 2
        self.limit_right_pos: float = self.config.rail_length_steps / 2
        
        # Serial interface emulation
        self._read_buffer = io.BytesIO()
        self._write_buffer = ""
        self._lock = threading.Lock()
        self.is_open = True
        
        # Timing
        self._last_physics_time = time.perf_counter()
        self._last_data_time = time.perf_counter()
        
        # Start physics thread
        self._running = True
        self._physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
        self._physics_thread.start()
        
        # Queue initial ready message
        self._queue_output("VELOCITY_MODE_READY\n")
        
        print("=== SIMULATOR MODE ===")
        print(f"Rail: {self.config.rail_length_steps} steps")
        print(f"Pendulum: {self.config.pendulum_length}m, {self.config.pendulum_mass}kg")
        print("======================")
    
    def _physics_loop(self):
        """Main physics simulation loop running in background thread"""
        while self._running:
            now = time.perf_counter()
            
            # Physics update
            dt = now - self._last_physics_time
            if dt >= self.config.physics_dt:
                self._update_physics(dt)
                self._last_physics_time = now
            
            # Sensor data output
            if now - self._last_data_time >= self.config.data_interval:
                self._send_sensor_data()
                self._last_data_time = now
            
            # Small sleep to prevent busy-waiting
            time.sleep(0.0005)
    
    def _update_physics(self, dt: float):
        """Update physics simulation for one timestep"""
        # Clamp dt to prevent instability
        dt = min(dt, 0.01)
        
        # --- Cart dynamics ---
        # Accelerate towards target velocity
        old_cart_velocity = self.cart_velocity
        velocity_error = self.target_velocity - self.cart_velocity
        max_accel_change = self.config.motor_accel * dt
        
        if abs(velocity_error) < max_accel_change:
            self.cart_velocity = self.target_velocity
        else:
            self.cart_velocity += math.copysign(max_accel_change, velocity_error)
        
        # Calculate ACTUAL cart acceleration (for pendulum coupling)
        # This is the real acceleration that was applied, not the desired one
        cart_accel = (self.cart_velocity - old_cart_velocity) / dt if dt > 0 else 0
        
        # Update cart position
        old_position = self.cart_position
        self.cart_position += self.cart_velocity * dt
        
        # Check limit switches
        limit_left = self.cart_position <= self.limit_left_pos
        limit_right = self.cart_position >= self.limit_right_pos
        
        if limit_left:
            self.cart_position = self.limit_left_pos
            if self.cart_velocity < 0:
                self.cart_velocity = 0
                if self.target_velocity < 0:
                    self.target_velocity = 0
        
        if limit_right:
            self.cart_position = self.limit_right_pos
            if self.cart_velocity > 0:
                self.cart_velocity = 0
                if self.target_velocity > 0:
                    self.target_velocity = 0
        
        # --- Pendulum dynamics ---
        # Convert cart acceleration from steps/s² to m/s²
        # For belt drive: ~80000 steps/meter typical
        # Lower value = stronger coupling (pendulum responds more to cart movement)
        steps_per_meter = 20000  # Reduced for stronger pendulum response
        cart_accel_ms2 = cart_accel / steps_per_meter
        
        # Pendulum equation of motion:
        # θ'' = -(g/L) * sin(θ) + (a_cart/L) * cos(θ) - c * θ'
        # Convention: θ=0 is DOWN (stable), θ=180° is UP (inverted/unstable)
        
        L = self.config.pendulum_length
        g = self.config.gravity
        c = self.config.damping
        
        # Gravity torque (negative because 0° is stable equilibrium)
        gravity_term = -(g / L) * math.sin(self.pendulum_angle)
        
        # Cart acceleration coupling (drives pendulum when cart moves)
        coupling_term = (cart_accel_ms2 / L) * math.cos(self.pendulum_angle)
        
        # Damping
        damping_term = c * self.pendulum_velocity
        
        # Angular acceleration
        angular_accel = gravity_term + coupling_term - damping_term
        
        # Update pendulum state (semi-implicit Euler)
        self.pendulum_velocity += angular_accel * dt
        self.pendulum_angle += self.pendulum_velocity * dt
        
        # Normalize angle to [0, 2π)
        while self.pendulum_angle < 0:
            self.pendulum_angle += 2 * math.pi
        while self.pendulum_angle >= 2 * math.pi:
            self.pendulum_angle -= 2 * math.pi
    
    def _send_sensor_data(self):
        """Generate sensor data in Arduino format"""
        # Convert angle to degrees (0-360, 180 = down)
        angle_deg = math.degrees(self.pendulum_angle)
        
        # Cart position (integer steps)
        position = int(self.cart_position)
        
        # Limit switches
        limit_left = 1 if self.cart_position <= self.limit_left_pos + 10 else 0
        limit_right = 1 if self.cart_position >= self.limit_right_pos - 10 else 0
        
        # Current velocity
        velocity = self.cart_velocity
        
        # Format: A:angle,P:position,LL:left,LR:right,V:velocity
        data = f"A:{angle_deg:.1f},P:{position},LL:{limit_left},LR:{limit_right},V:{velocity:.2f}\n"
        self._queue_output(data)
    
    def _queue_output(self, data: str):
        """Add data to the read buffer"""
        with self._lock:
            # Append to buffer
            current_pos = self._read_buffer.tell()
            self._read_buffer.seek(0, 2)  # Seek to end
            self._read_buffer.write(data.encode())
            self._read_buffer.seek(current_pos)
    
    def _parse_command(self, cmd: str):
        """Parse and execute a command from the controller"""
        cmd = cmd.strip()
        if not cmd:
            return
        
        if cmd.startswith('V'):
            # Velocity command
            try:
                self.target_velocity = float(cmd[1:])
            except ValueError:
                pass
        
        elif cmd == 'S':
            # Stop
            self.target_velocity = 0
        
        elif cmd == 'Z':
            # Zero position
            # Adjust limit positions
            self.limit_left_pos -= self.cart_position
            self.limit_right_pos -= self.cart_position
            self.cart_position = 0
            self._queue_output("ZEROED\n")
    
    # --- Serial-like interface ---
    
    def write(self, data: bytes) -> int:
        """Write data (receive commands from controller)"""
        self._write_buffer += data.decode()
        
        # Process complete lines
        while '\n' in self._write_buffer:
            line, self._write_buffer = self._write_buffer.split('\n', 1)
            self._parse_command(line)
        
        return len(data)
    
    def read(self, size: int = 1) -> bytes:
        """Read data (send sensor data to controller)"""
        with self._lock:
            data = self._read_buffer.read(size)
            
            # Compact buffer if we've read a lot
            if self._read_buffer.tell() > 10000:
                remaining = self._read_buffer.read()
                self._read_buffer = io.BytesIO(remaining)
            
            return data
    
    def readline(self) -> bytes:
        """Read a complete line"""
        with self._lock:
            line = b""
            while True:
                char = self._read_buffer.read(1)
                if not char:
                    # Put back partial line
                    self._read_buffer.seek(-len(line), 1) if line else None
                    return b""
                line += char
                if char == b'\n':
                    return line
    
    @property
    def in_waiting(self) -> int:
        """Number of bytes waiting to be read"""
        with self._lock:
            current_pos = self._read_buffer.tell()
            self._read_buffer.seek(0, 2)
            end_pos = self._read_buffer.tell()
            self._read_buffer.seek(current_pos)
            return end_pos - current_pos
    
    def close(self):
        """Stop the simulator"""
        self._running = False
        self.is_open = False
        if self._physics_thread.is_alive():
            self._physics_thread.join(timeout=1.0)
    
    def reset_input_buffer(self):
        """Clear input buffer"""
        with self._lock:
            self._read_buffer = io.BytesIO()
    
    def reset_output_buffer(self):
        """Clear output buffer"""
        self._write_buffer = ""
    
    # --- Additional methods for testing ---
    
    def set_pendulum_angle(self, degrees: float):
        """Set pendulum angle in degrees (180 = down, 0 = up)"""
        self.pendulum_angle = math.radians(degrees)
        self.pendulum_velocity = 0
    
    def give_pendulum_push(self, velocity_deg_per_sec: float):
        """Give the pendulum an angular velocity push"""
        self.pendulum_velocity = math.radians(velocity_deg_per_sec)
    
    def get_state(self) -> dict:
        """Get current simulation state"""
        return {
            'cart_position': self.cart_position,
            'cart_velocity': self.cart_velocity,
            'target_velocity': self.target_velocity,
            'pendulum_angle_deg': math.degrees(self.pendulum_angle),
            'pendulum_velocity_deg': math.degrees(self.pendulum_velocity),
            'limit_left': self.cart_position <= self.limit_left_pos + 10,
            'limit_right': self.cart_position >= self.limit_right_pos - 10,
        }


# Test the simulator standalone
if __name__ == "__main__":
    print("Testing Pendulum Simulator...")
    
    sim = PendulumSimulator()
    
    # Give pendulum a push
    sim.give_pendulum_push(50)
    
    # Move cart right
    sim.write(b"V1000\n")
    
    try:
        while True:
            # Read and print sensor data
            while sim.in_waiting > 0:
                line = sim.readline()
                if line:
                    print(f"Sensor: {line.decode().strip()}")
            
            time.sleep(0.1)
            
    except KeyboardInterrupt:
        print("\nStopping...")
        sim.close()

