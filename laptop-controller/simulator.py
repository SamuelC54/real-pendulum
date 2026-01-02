"""
Inverted Pendulum Hardware Simulator

Simulates the physical system:
- Cart on a linear rail with stepper motor
- Pendulum attached to cart with rotary encoder
- Limit switches at both ends

Physics model:
- Cart: driven by acceleration commands only
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
    
    def __init__(self, config: Optional[SimulatorConfig] = None, start_background_thread: bool = True):
        self.config = config or SimulatorConfig()
        
        # Cart state (in steps)
        self.cart_position: float = 0.0          # Current position (steps)
        self.cart_velocity: float = 0.0          # Current velocity (steps/s)
        self.cart_acceleration: float = 0.0      # Current acceleration (steps/s²)
        self.target_acceleration: float = 0.0    # Commanded acceleration (steps/s²)
        
        # Pendulum state (0° = DOWN/stable, 180° = UP/inverted)
        # Start at 90° (horizontal) so it falls to 0° (down)
        self.pendulum_angle: float = math.pi / 2  # Radians (90° = horizontal)
        self.pendulum_velocity: float = 0.0       # rad/s
        self.pendulum_acceleration: float = 0.0   # rad/s²
        
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
        
        # Start physics thread (only if requested, e.g., for controller use)
        self._running = start_background_thread
        if start_background_thread:
            self._physics_thread = threading.Thread(target=self._physics_loop, daemon=True)
            self._physics_thread.start()
            # Queue initial ready message
            self._queue_output("ACCELERATION_MODE_READY\n")
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
        
        # --- Cart dynamics (acceleration-only control) ---
        # Clamp acceleration to motor limits
        target_accel = max(-self.config.motor_accel, min(self.config.motor_accel, self.target_acceleration))
        
        # Update velocity based on acceleration
        self.cart_velocity += target_accel * dt
        
        # Clamp velocity to max speed
        max_speed = 50000  # Maximum cart speed (steps/s)
        self.cart_velocity = max(-max_speed, min(max_speed, self.cart_velocity))
        
        # Store actual cart acceleration (for pendulum coupling)
        self.cart_acceleration = target_accel
        
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
            # Stop acceleration if trying to move left
            if self.target_acceleration < 0:
                self.target_acceleration = 0
        
        if limit_right:
            self.cart_position = self.limit_right_pos
            if self.cart_velocity > 0:
                self.cart_velocity = 0
            # Stop acceleration if trying to move right
            if self.target_acceleration > 0:
                self.target_acceleration = 0
        
        # --- Pendulum dynamics ---
        # Convert cart acceleration from steps/s² to m/s²
        # For belt drive: ~80000 steps/meter typical
        # Lower value = stronger coupling (pendulum responds more to cart movement)
        steps_per_meter = 20000  # Reduced for stronger pendulum response
        cart_accel_ms2 = target_accel / steps_per_meter
        
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
        self.pendulum_acceleration = angular_accel  # Store for get_state()
        
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
        
        if cmd == 'S':
            # Stop immediately (set both acceleration and velocity to zero)
            self.target_acceleration = 0.0
            self.cart_velocity = 0.0
        
        elif cmd == 'Z':
            # Zero position
            # Adjust limit positions
            self.limit_left_pos -= self.cart_position
            self.limit_right_pos -= self.cart_position
            self.cart_position = 0
            self._queue_output("ZEROED\n")
        
        # Note: Velocity commands ('V') are no longer supported - use acceleration only
    
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
    
    def set_pendulum_upright(self):
        """Reset pendulum to upright position (180°) with small perturbation"""
        with self._lock:
            import random
            perturbation = random.uniform(-5, 5)  # ±5 degrees
            self.pendulum_angle = math.radians(180 + perturbation)
            self.pendulum_velocity = random.uniform(-0.3, 0.3)  # Small initial velocity
            print(f"Simulator: Pendulum set to {180 + perturbation:.1f}°")
    
    # --- Training interface (manual physics stepping) ---
    
    def set_state(self, cart_position: float = None, cart_velocity: float = None,
                  pendulum_angle: float = None, pendulum_velocity: float = None):
        """Set simulator state directly (for training)"""
        with self._lock:
            if cart_position is not None:
                self.cart_position = cart_position
            if cart_velocity is not None:
                self.cart_velocity = cart_velocity
            if pendulum_angle is not None:
                self.pendulum_angle = pendulum_angle
            if pendulum_velocity is not None:
                self.pendulum_velocity = pendulum_velocity
    
    def get_state(self):
        """Get current simulator state (for training)"""
        try:
            if hasattr(self, '_lock') and self._lock is not None:
                with self._lock:
                    return {
                        'cart_position': self.cart_position,
                        'cart_velocity': self.cart_velocity,
                        'cart_acceleration': self.cart_acceleration,
                        'pendulum_angle': self.pendulum_angle,
                        'pendulum_velocity': self.pendulum_velocity,
                        'pendulum_acceleration': self.pendulum_acceleration,
                        'limit_left': self.cart_position <= self.limit_left_pos,
                        'limit_right': self.cart_position >= self.limit_right_pos,
                        'limit_left_pos': self.limit_left_pos,
                        'limit_right_pos': self.limit_right_pos,
                    }
            else:
                # Fallback if lock not initialized
                return {
                    'cart_position': self.cart_position,
                    'cart_velocity': self.cart_velocity,
                    'cart_acceleration': self.cart_acceleration,
                    'pendulum_angle': self.pendulum_angle,
                    'pendulum_velocity': self.pendulum_velocity,
                    'pendulum_acceleration': self.pendulum_acceleration,
                    'limit_left': self.cart_position <= self.limit_left_pos,
                    'limit_right': self.cart_position >= self.limit_right_pos,
                    'limit_left_pos': self.limit_left_pos,
                    'limit_right_pos': self.limit_right_pos,
                    }
        except Exception as e:
            # Emergency fallback - return current values without lock
            return {
                'cart_position': getattr(self, 'cart_position', 0.0),
                'cart_velocity': getattr(self, 'cart_velocity', 0.0),
                'pendulum_angle': getattr(self, 'pendulum_angle', 0.0),
                'pendulum_velocity': getattr(self, 'pendulum_velocity', 0.0),
                'limit_left': False,
                'limit_right': False,
                'limit_left_pos': getattr(self, 'limit_left_pos', -3750.0),
                'limit_right_pos': getattr(self, 'limit_right_pos', 3750.0),
            }
    
    def step(self, dt: float):
        """
        Manually step physics simulation (for training).
        Call this instead of letting the background thread run.
        """
        with self._lock:
            self._update_physics(dt)
    
    def set_target_acceleration(self, acceleration: float):
        """Set target acceleration for the cart (steps/s²) - acceleration-only control"""
        with self._lock:
            self.target_acceleration = acceleration
    
    def stop(self):
        """Immediately stop the cart (set both velocity and acceleration to zero)"""
        with self._lock:
            self.target_acceleration = 0.0
            self.cart_velocity = 0.0
    
    def close(self):
        """Stop the simulator"""
        self._running = False
        self.is_open = False
        if hasattr(self, '_physics_thread') and self._physics_thread.is_alive():
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
    
    # Removed duplicate get_state() method - using the one above with proper keys


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

