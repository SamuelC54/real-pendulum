"""
Test different LQR tuning combinations to find the best parameters
that keep the pendulum balanced the longest.
"""

import math
import time
import numpy as np
from simulator import PendulumSimulator, SimulatorConfig
from lqr_controller import compute_lqr_gain, compute_lqr_control

# Test parameters
TEST_DURATION = 15.0  # seconds per test (increased to see longer-term stability)
PHYSICS_DT = 0.001
CONTROL_DT = 0.01  # 100Hz control rate

# Test combinations
Q_ANGLE_RANGE = [10.0, 20.0, 30.0, 50.0]
Q_ANGLE_VEL_RANGE = [1.0, 2.0, 5.0]
Q_POSITION_RANGE = [30.0, 50.0, 100.0, 150.0]
Q_VELOCITY_RANGE = [0.1, 0.2, 0.5]
R_CONTROL_RANGE = [3.0, 5.0, 10.0, 15.0]

# Reduced combinations for faster testing (can expand later)
TEST_COMBINATIONS = [
    # Format: (q_angle, q_angle_vel, q_position, q_velocity, r_control)
    (20.0, 2.0, 50.0, 0.2, 5.0),  # Current default
    (30.0, 2.0, 50.0, 0.2, 5.0),  # Higher angle penalty
    (20.0, 2.0, 100.0, 0.2, 5.0),  # Higher position penalty
    (20.0, 2.0, 50.0, 0.2, 10.0),  # Higher control penalty (less aggressive)
    (30.0, 2.0, 100.0, 0.2, 5.0),  # Higher angle + position
    (20.0, 5.0, 50.0, 0.2, 5.0),  # Higher angular velocity penalty
    (30.0, 2.0, 100.0, 0.2, 10.0),  # Higher penalties, less aggressive
    (50.0, 2.0, 150.0, 0.2, 5.0),  # Very high state penalties
    (20.0, 2.0, 50.0, 0.5, 5.0),  # Higher velocity penalty
    (30.0, 5.0, 100.0, 0.2, 10.0),  # Balanced high penalties
    (40.0, 3.0, 80.0, 0.3, 8.0),  # Medium-high all around
    (25.0, 2.5, 75.0, 0.25, 7.0),  # Balanced medium
    (35.0, 3.0, 120.0, 0.2, 6.0),  # High position focus
    (45.0, 2.0, 100.0, 0.2, 12.0),  # High angle, less aggressive
    (20.0, 2.0, 200.0, 0.2, 5.0),  # Very high position penalty
    # Additional high position penalty tests
    (35.0, 3.0, 200.0, 0.2, 6.0),  # Best angle params + very high position
    (35.0, 3.0, 150.0, 0.2, 6.0),  # Best angle params + high position
    (30.0, 2.5, 180.0, 0.2, 5.0),  # Balanced with very high position
    (40.0, 3.0, 200.0, 0.3, 7.0),  # High all around
    (35.0, 3.0, 250.0, 0.2, 6.0),  # Extreme position penalty
    (30.0, 3.0, 250.0, 0.25, 8.0),  # Extreme position, higher R
    (40.0, 4.0, 200.0, 0.3, 8.0),  # Very high all penalties
    (35.0, 3.0, 300.0, 0.2, 6.0),  # Very extreme position
    (40.0, 3.5, 250.0, 0.25, 7.0),  # High all around
    (30.0, 3.0, 400.0, 0.2, 8.0),  # Extreme position focus
    (35.0, 4.0, 300.0, 0.3, 8.0),  # Very high penalties
]

# Use the LQR controller module instead of reimplementing

def test_lqr_combination(q_angle, q_angle_vel, q_position, q_velocity, r_control):
    """Test a specific LQR combination and return survival time and stats"""
    print(f"\nTesting: Q=[{q_angle:.1f}, {q_angle_vel:.1f}, {q_position:.1f}, {q_velocity:.1f}], R=[{r_control:.1f}]")
    
    # Create simulator
    config = SimulatorConfig()
    sim = PendulumSimulator(config, start_background_thread=False)
    
    # Initialize: pendulum upright with small perturbation, cart centered, zero velocity
    import random
    center_pos = 0.0
    # Add small random perturbation to angle (like real system)
    # Reduced perturbation for more realistic starting conditions
    angle_perturbation = random.uniform(-3, 3)  # ±3 degrees (was ±5)
    velocity_perturbation = random.uniform(-0.2, 0.2)  # Small initial angular velocity (was ±0.3)
    sim.set_state(
        cart_position=center_pos,
        cart_velocity=0.0,
        pendulum_angle=math.radians(180.0 + angle_perturbation),
        pendulum_velocity=velocity_perturbation
    )
    
    # Compute LQR gain using the module with physics parameters from config
    K = compute_lqr_gain(
        q_angle=q_angle,
        q_angle_vel=q_angle_vel,
        q_position=q_position,
        q_velocity=q_velocity,
        q_integral=10.0,  # Default integral weight for LQI
        r_control=r_control,
        cart_mass=config.cart_mass,
        pendulum_mass=config.pendulum_mass,
        pendulum_length=config.pendulum_length,
        gravity=config.gravity,
        damping=config.damping,
        cart_friction=config.cart_friction,
        max_angular_vel=10.0,  # rad/s for normalization
        use_lqi=True,  # Use LQI for integral action
        use_cache=False  # Don't cache for testing different parameters
    )
    if K is None:
        print(f"  ERROR: Failed to compute LQR gain")
        return {
            'survival_time': 0.0,
            'failed': True,
            'max_angle_deviation': 0.0,
            'max_position_deviation': 0.0,
            'avg_angle_error': 0.0,
            'avg_position_error': 0.0,
            'params': (q_angle, q_angle_vel, q_position, q_velocity, r_control)
        }
    print(f"  LQR Gain K = {K}")
    
    # Test parameters
    rail_length = config.rail_length_steps
    rail_half = rail_length / 2
    MAX_SPEED = 50000  # Increased for better performance
    MAX_ACCEL = 50000  # steps/s²
    
    # Tracking
    start_time = time.perf_counter()
    last_control_time = start_time
    survival_time = 0.0
    failed = False
    
    # Failure criteria
    ANGLE_TOLERANCE = math.radians(45.0)  # ±45° from upright
    POSITION_TOLERANCE = rail_half * 0.8  # 80% of rail half-length
    MAX_ANGLE_DEVIATION = math.radians(90.0)  # ±90° = completely fallen
    
    # Statistics
    max_angle_deviation = 0.0
    max_position_deviation = 0.0
    total_angle_error = 0.0
    total_position_error = 0.0
    step_count = 0
    
    # Run simulation
    while time.perf_counter() - start_time < TEST_DURATION:
        now = time.perf_counter()
        
        # Physics update
        dt = min(now - sim._last_physics_time, 0.01)
        if dt >= config.physics_dt:
            sim._update_physics(dt)
            sim._last_physics_time = now
        
        # Control update (100Hz)
        if now - last_control_time >= CONTROL_DT:
            # Get current state
            angle_rad = sim.pendulum_angle
            angle_rad = angle_rad % (2 * math.pi)
            angle_deviation = angle_rad - math.pi
            
            # Wrap angle deviation
            if angle_deviation > math.pi:
                angle_deviation -= 2 * math.pi
            elif angle_deviation < -math.pi:
                angle_deviation += 2 * math.pi
            
            angular_vel_rad = sim.pendulum_velocity
            
            # Position from center
            position_from_center = sim.cart_position - center_pos
            cart_pos_normalized = position_from_center / rail_half if rail_half > 0 else 0.0
            cart_vel_normalized = sim.cart_velocity / MAX_SPEED if MAX_SPEED > 0 else 0.0
            
            # Use LQR controller module with LQI
            # Initialize position integral for LQI
            if 'position_integral' not in locals():
                position_integral = 0.0
            
            acceleration_steps, at_limit, position_integral = compute_lqr_control(
                angle_deg=math.degrees(angle_rad),
                angular_velocity_deg_s=math.degrees(angular_vel_rad),
                position=sim.cart_position,
                velocity=sim.cart_velocity,
                limit_left_pos=sim.limit_left_pos,
                limit_right_pos=sim.limit_right_pos,
                K=K,
                max_accel=config.motor_accel,  # Use simulator's motor acceleration limit
                use_lqi=True,  # Use LQI for integral action
                position_integral=position_integral,
                dt=CONTROL_DT,  # Control timestep
            )
            
            # Set acceleration
            sim.set_target_acceleration(float(acceleration_steps))
            
            # Check failure conditions
            angle_dev_deg = abs(math.degrees(angle_deviation))
            pos_dev = abs(position_from_center)
            
            if angle_dev_deg > 90.0 or pos_dev > POSITION_TOLERANCE:
                survival_time = now - start_time
                failed = True
                print(f"  X Failed at {survival_time:.2f}s - Angle: {angle_dev_deg:.1f}deg, Position: {pos_dev:.0f}")
                break
            
            # Update statistics
            max_angle_deviation = max(max_angle_deviation, angle_dev_deg)
            max_position_deviation = max(max_position_deviation, pos_dev)
            total_angle_error += abs(angle_deviation)
            total_position_error += abs(position_from_center)
            step_count += 1
            
            last_control_time = now
        
        # Small sleep
        time.sleep(0.001)
    
    if not failed:
        survival_time = TEST_DURATION
        print(f"  OK Survived full {TEST_DURATION:.1f}s test")
    
    # Calculate average errors
    avg_angle_error = math.degrees(total_angle_error / step_count) if step_count > 0 else 0.0
    avg_position_error = total_position_error / step_count if step_count > 0 else 0.0
    
    # Debug: print if no steps were counted (indicates a problem)
    if step_count == 0:
        print(f"  WARNING: No control steps were executed!")
    
    sim._running = False
    
    return {
        'survival_time': survival_time,
        'failed': failed,
        'max_angle_deviation': max_angle_deviation,
        'max_position_deviation': max_position_deviation,
        'avg_angle_error': avg_angle_error,
        'avg_position_error': avg_position_error,
        'params': (q_angle, q_angle_vel, q_position, q_velocity, r_control)
    }

def main():
    """Test all combinations and find the best"""
    print("=" * 70)
    print("LQR Tuning Test - Finding Best Parameters")
    print("=" * 70)
    print(f"Testing {len(TEST_COMBINATIONS)} combinations")
    print(f"Test duration: {TEST_DURATION}s per combination")
    print(f"Failure criteria: Angle > 90° or Position > 80% of rail")
    print("=" * 70)
    
    results = []
    
    for i, (q_angle, q_angle_vel, q_position, q_velocity, r_control) in enumerate(TEST_COMBINATIONS, 1):
        print(f"\n[{i}/{len(TEST_COMBINATIONS)}] ", end="")
        result = test_lqr_combination(q_angle, q_angle_vel, q_position, q_velocity, r_control)
        results.append(result)
        
        # Print summary
        print(f"  Summary: Survival={result['survival_time']:.2f}s, "
              f"MaxAngle={result['max_angle_deviation']:.1f}°, "
              f"MaxPos={result['max_position_deviation']:.0f}, "
              f"AvgAngle={result['avg_angle_error']:.2f}°, "
              f"AvgPos={result['avg_position_error']:.0f}")
    
    # Sort by survival time (longest first)
    results.sort(key=lambda x: x['survival_time'], reverse=True)
    
    # Print results
    print("\n" + "=" * 70)
    print("RESULTS - Ranked by Survival Time")
    print("=" * 70)
    
    for i, result in enumerate(results[:10], 1):  # Top 10
        q_angle, q_angle_vel, q_position, q_velocity, r_control = result['params']
        status = "OK" if not result['failed'] else "X"
        print(f"\n{i}. {status} Survival: {result['survival_time']:.2f}s")
        print(f"   Q=[{q_angle:.1f}, {q_angle_vel:.1f}, {q_position:.1f}, {q_velocity:.1f}], R=[{r_control:.1f}]")
        print(f"   Max Angle: {result['max_angle_deviation']:.1f}°, Max Position: {result['max_position_deviation']:.0f}")
        print(f"   Avg Angle: {result['avg_angle_error']:.2f}°, Avg Position: {result['avg_position_error']:.0f}")
    
    # Best result
    best = results[0]
    q_angle, q_angle_vel, q_position, q_velocity, r_control = best['params']
    print("\n" + "=" * 70)
    print("BEST COMBINATION:")
    print("=" * 70)
    print(f"lqr_q_angle = {q_angle:.1f}")
    print(f"lqr_q_angle_vel = {q_angle_vel:.1f}")
    print(f"lqr_q_position = {q_position:.1f}")
    print(f"lqr_q_velocity = {q_velocity:.1f}")
    print(f"lqr_r_control = {r_control:.1f}")
    print(f"\nSurvival time: {best['survival_time']:.2f}s")
    print(f"Max angle deviation: {best['max_angle_deviation']:.1f}°")
    print(f"Max position deviation: {best['max_position_deviation']:.0f} steps")
    print("=" * 70)

if __name__ == "__main__":
    main()

