"""
LQR (Linear Quadratic Regulator) controller for inverted pendulum.

Implements LQR control for balancing the pendulum in the upright position.
Improved version with:
- Consistent state normalization
- LQI (LQR with Integral) for position
- Parameter-based gain caching
- Direct physical unit control
"""

import math
import hashlib
import numpy as np
from control import lqr
from typing import Optional, Tuple
from physics import compute_linearized_matrices


# LQR gain cache with parameter-based invalidation
_lqr_gain: Optional[np.ndarray] = None
_lqr_gain_params_hash: Optional[str] = None


def reset_lqr_gain():
    """Reset LQR gain to force recomputation (useful after parameter changes)"""
    global _lqr_gain, _lqr_gain_params_hash
    _lqr_gain = None
    _lqr_gain_params_hash = None


def _compute_params_hash(
    q_angle: float, q_angle_vel: float, q_position: float, q_velocity: float,
    q_integral: float, r_control: float,
    cart_mass: float, pendulum_mass: float, pendulum_length: float,
    gravity: float, damping: float, cart_friction: float,
    max_angular_vel: float, use_lqi: bool
) -> str:
    """Compute hash of parameters for cache invalidation"""
    params_str = f"{q_angle:.6f},{q_angle_vel:.6f},{q_position:.6f},{q_velocity:.6f},{q_integral:.6f},{r_control:.6f},{cart_mass:.6f},{pendulum_mass:.6f},{pendulum_length:.6f},{gravity:.6f},{damping:.6f},{cart_friction:.6f},{max_angular_vel:.6f},{use_lqi}"
    return hashlib.md5(params_str.encode()).hexdigest()


def compute_lqr_gain(
    q_angle: float = 20.0,
    q_angle_vel: float = 2.0,
    q_position: float = 100.0,
    q_velocity: float = 0.2,
    q_integral: float = 10.0,  # Integral weight for LQI
    r_control: float = 5.0,
    cart_mass: float = 0.5,
    pendulum_mass: float = 0.1,
    pendulum_length: float = 0.3,
    gravity: float = 9.81,
    damping: float = 0.2,
    cart_friction: float = 0.1,
    max_angular_vel: float = 10.0,  # rad/s for normalization
    use_lqi: bool = True,  # Use LQI (LQR with Integral) for position
    use_cache: bool = True
) -> Optional[np.ndarray]:
    """
    Compute LQR/LQI gain matrix using control library.
    
    Uses the physics model to compute linearized matrices, ensuring consistency
    between the controller and simulator.
    
    Linearizes around upright equilibrium (θ = π).
    State (normalized): [(θ-π)/π, θ_dot/max_angular_vel, x_normalized, x_dot_normalized, ∫x_normalized]
    
    Args:
        q_angle: Q matrix weight for normalized angle penalty
        q_angle_vel: Q matrix weight for normalized angular velocity penalty
        q_position: Q matrix weight for normalized position penalty
        q_velocity: Q matrix weight for normalized velocity penalty
        q_integral: Q matrix weight for position integral (LQI only)
        r_control: R matrix weight for control effort penalty
        cart_mass: Cart mass (kg) - matches physics model
        pendulum_mass: Pendulum mass (kg) - matches physics model
        pendulum_length: Pendulum length (m)
        gravity: Gravity acceleration (m/s²)
        damping: Angular damping coefficient
        cart_friction: Cart friction coefficient (N·s/m) - matches physics model
        max_angular_vel: Maximum angular velocity for normalization (rad/s)
        use_lqi: If True, use LQI (adds integral state for position)
        use_cache: If True, use cached gain if available
    
    Returns:
        LQR/LQI gain matrix K (1D array) or None if computation fails
    """
    global _lqr_gain, _lqr_gain_params_hash
    
    # Compute parameter hash for cache invalidation
    params_hash = _compute_params_hash(
        q_angle, q_angle_vel, q_position, q_velocity, q_integral, r_control,
        cart_mass, pendulum_mass, pendulum_length, gravity, damping, cart_friction,
        max_angular_vel, use_lqi
    )
    
    if use_cache and _lqr_gain is not None and _lqr_gain_params_hash == params_hash:
        return _lqr_gain
    
    # Use physics module to compute linearized matrices (in physical units)
    # This ensures the LQR model matches the actual physics
    A_base, B_base = compute_linearized_matrices(
        cart_mass=cart_mass,
        pendulum_mass=pendulum_mass,
        pendulum_length=pendulum_length,
        gravity=gravity,
        damping=damping,
        cart_friction=cart_friction,
    )
    
    # CRITICAL FIX: The A and B matrices are in physical units, but we use normalized states
    # We need to normalize A and B to match the normalized state space
    # 
    # Physical state: x_phys = [θ-π (rad), θ_dot (rad/s), x (m), x_dot (m/s)]
    # Normalized state: x_norm = [(θ-π)/π, θ_dot/max_angular_vel, x/rail_half, x_dot/max_speed]
    #
    # Transformation: x_norm = T @ x_phys, where T = diag([1/π, 1/max_angular_vel, 1/rail_half, 1/max_speed])
    # Normalized system: A_norm = T @ A_phys @ T^-1, B_norm = T @ B_phys
    
    # Use reference values for normalization (position/velocity use actual values in control function)
    # For A/B normalization, we need consistent reference values
    steps_per_meter = 20000.0
    rail_half_ref_steps = 3750.0  # Reference: half of 7500 step rail
    max_speed_ref_steps = 50000.0  # Reference max speed
    rail_half_ref_m = rail_half_ref_steps / steps_per_meter  # meters
    max_speed_ref_ms = max_speed_ref_steps / steps_per_meter  # m/s
    
    # Normalization transformation matrix T (diagonal)
    # T = diag([1/π, 1/max_angular_vel, 1/rail_half_m, 1/max_speed_ms])
    T = np.array([
        [1.0 / math.pi, 0, 0, 0],
        [0, 1.0 / max_angular_vel, 0, 0],
        [0, 0, 1.0 / rail_half_ref_m, 0],
        [0, 0, 0, 1.0 / max_speed_ref_ms]
    ])
    T_inv = np.linalg.inv(T)
    
    # Normalize A and B matrices
    A_norm = T @ A_base @ T_inv
    B_norm = T @ B_base
    
    if use_lqi:
        # Extend system for LQI: add integral state for position
        # Extended normalized state: [(θ-π)/π, θ_dot/max_angular_vel, x_norm, x_dot_norm, ∫x_norm]
        n = 5
        A = np.zeros((n, n))
        A[:4, :4] = A_norm
        # Integral dynamics: d(∫x_norm)/dt = x_norm
        # Since both are normalized, this is just 1.0
        A[4, 2] = 1.0  # d(∫x_norm)/dt = x_norm
        
        B = np.zeros((n, 1))
        B[:4, 0] = B_norm[:, 0]
        B[4, 0] = 0.0  # Integral doesn't directly depend on control
        
        # Cost matrices: Q penalizes state deviations, R penalizes control effort
        # All states are normalized, so Q weights are directly comparable
        Q = np.diag([q_angle, q_angle_vel, q_position, q_velocity, q_integral])
    else:
        # Standard LQR without integral (normalized)
        A = A_norm
        B = B_norm
        Q = np.diag([q_angle, q_angle_vel, q_position, q_velocity])
    
    R = np.array([[r_control]])
    
    try:
        # Compute LQR/LQI gain
        K, _, _ = lqr(A, B, Q, R)
        K = K[0] if K.ndim == 2 else K  # Flatten to 1D
        
        if use_cache:
            _lqr_gain = K
            _lqr_gain_params_hash = params_hash
            controller_type = "LQI" if use_lqi else "LQR"
            print(f"{controller_type} gain: K = {K}", flush=True)
        
        return K
    except Exception as e:
        print(f"LQR gain computation failed: {e}", flush=True)
        return None


def compute_lqr_control(
    angle_deg: float,
    angular_velocity_deg_s: float,
    position: float,
    velocity: float,
    limit_left_pos: float,
    limit_right_pos: float,
    K: Optional[np.ndarray],
    max_accel: float = 500000,  # Match simulator motor_accel limit
    max_speed: float = 50000,  # Increased for better performance
    max_angular_vel: float = 10.0,  # rad/s for normalization
    use_lqi: bool = True,  # Use LQI (LQR with Integral)
    position_integral: float = 0.0,  # Integral of position error (for LQI)
    soft_limit_threshold: float = 0.5,  # Start soft limits at this normalized position
    dt: float = 0.01,  # Control timestep for integral update
) -> Tuple[int, bool, float]:
    """
    Compute LQR/LQI control command for balancing the pendulum.
    
    Improved implementation with:
    - Consistent state normalization (all states normalized)
    - LQI support (integral action for position)
    - Direct physical unit control (no arbitrary scaling)
    - Configurable soft limits
    
    Args:
        angle_deg: Current pendulum angle (degrees, 0°=DOWN, 180°=UP)
        angular_velocity_deg_s: Current angular velocity (degrees/s)
        position: Current cart position (steps)
        velocity: Current cart velocity (steps/s)
        limit_left_pos: Left limit position (steps)
        limit_right_pos: Right limit position (steps)
        K: LQR/LQI gain matrix (1D array, 4 or 5 elements)
        max_accel: Maximum acceleration (steps/s²)
        max_speed: Maximum speed for normalization (steps/s)
        max_angular_vel: Maximum angular velocity for normalization (rad/s)
        use_lqi: If True, K has 5 elements and uses integral state
        position_integral: Current value of position integral (for LQI)
        soft_limit_threshold: Normalized position threshold for soft limits (0.0-1.0)
        dt: Control timestep for integral update (s)
    
    Returns:
        Tuple of (acceleration_steps, limit_hit, new_position_integral)
        acceleration_steps: Commanded acceleration (steps/s²)
        limit_hit: True if at a limit switch
        new_position_integral: Updated integral value (for next call)
    """
    if K is None:
        return 0, False, position_integral
    
    # System convention: 0° = DOWN (stable), 180° = UP (inverted/unstable)
    # LQR is linearized around 180° (π) as upright equilibrium
    angle_rad = math.radians(angle_deg)
    # Normalize angle to [0, 2π) and compute deviation from upright (π)
    angle_rad = angle_rad % (2 * math.pi)
    angle_deviation_rad = angle_rad - math.pi  # Deviation from upright (π)
    
    # Wrap angle deviation to [-π, π] for better control near boundaries
    if angle_deviation_rad > math.pi:
        angle_deviation_rad -= 2 * math.pi
    elif angle_deviation_rad < -math.pi:
        angle_deviation_rad += 2 * math.pi
    
    # Angular velocity in rad/s
    angular_vel_rad = math.radians(angular_velocity_deg_s)
    
    # Get rail length for position normalization
    rail_length = limit_right_pos - limit_left_pos
    if rail_length <= 0:
        # Fallback if limits not set yet (use default 7500 steps)
        rail_length = 7500
    rail_half = rail_length / 2
    
    # Calculate center position (where we want to stay)
    center_pos = (limit_left_pos + limit_right_pos) / 2
    
    # Position deviation from center (positive = right of center, negative = left of center)
    position_from_center = position - center_pos
    
    # PRIORITY 1: Normalize ALL states consistently
    # - Angle: normalize by π (so ±1 = ±180°)
    # - Angular velocity: normalize by max_angular_vel
    # - Position: normalize by rail_half (already done)
    # - Velocity: normalize by max_speed (already done)
    angle_normalized = angle_deviation_rad / math.pi  # [-1, 1] for ±180°
    angular_vel_normalized = angular_vel_rad / max_angular_vel if max_angular_vel > 0 else 0.0
    cart_pos_normalized = position_from_center / rail_half if rail_half > 0 else 0.0
    cart_vel_normalized = velocity / max_speed if max_speed > 0 else 0.0
    
    # PRIORITY 2: Add integral state for LQI
    # Update position integral (integral of normalized position error)
    # IMPORTANT: Use the same sign convention as the position state (negated)
    if use_lqi:
        # Integrate the negated normalized position (same as used in state vector)
        # This ensures the integral has the correct sign for the control law
        position_error_normalized = -cart_pos_normalized  # Negated for control direction
        new_position_integral = position_integral + position_error_normalized * dt
        # Anti-windup: clamp integral to prevent excessive accumulation
        max_integral = 2.0  # Maximum integral value (2 seconds of full error)
        new_position_integral = max(-max_integral, min(max_integral, new_position_integral))
    else:
        new_position_integral = 0.0
    
    # State vector: ALL states normalized consistently
    # Note: Negate position so that positive position (right of center) 
    #       results in negative control (move left toward center)
    if use_lqi and len(K) == 5:
        # LQI: extended state with integral
        x = np.array([
            angle_normalized,           # Normalized angle deviation [-1, 1]
            angular_vel_normalized,     # Normalized angular velocity
            -cart_pos_normalized,       # Negated normalized position
            cart_vel_normalized,        # Normalized velocity
            new_position_integral       # Integral of negated normalized position error
        ])
    else:
        # Standard LQR: 4 states
        x = np.array([
            angle_normalized,           # Normalized angle deviation [-1, 1]
            angular_vel_normalized,     # Normalized angular velocity
            -cart_pos_normalized,       # Negated normalized position
            cart_vel_normalized         # Normalized velocity
        ])
    
    # LQR control law: u = -K * x (standard form)
    u = -K @ x  # Standard LQR form (negative feedback)
    u_normalized = u[0] if u.ndim > 0 else float(u)  # Extract scalar
    
    # PRIORITY 4: Convert normalized control output to physical units
    # The control output u_normalized is from the normalized LQR system
    # 
    # For normalized states, the control output u_norm has units that depend on B_norm
    # B_norm = T @ B_phys, where T normalizes the states
    # The control input to the normalized system is also normalized
    #
    # To convert back to physical units:
    # - The normalized B matrix scales control by the normalization factors
    # - For velocity (x_dot), normalization is by max_speed_ms
    # - So u_norm needs to be scaled by max_speed_ms to get physical acceleration
    #
    # Actually, since B[3,0] = 1 in physical units (direct effect on velocity),
    # and B_norm[3,0] = 1/max_speed_ms after normalization,
    # the control output u_norm is in units of (m/s²) * max_speed_ms
    # So: u_phys = u_norm * max_speed_ms
    #
    # But wait - we need to think about this more carefully.
    # The normalized system: x_norm' = A_norm @ x_norm + B_norm @ u_norm
    # where u_norm is the normalized control input
    # The physical system: x_phys' = A_phys @ x_phys + B_phys @ u_phys
    #
    # For the control output from LQR: u_norm = -K_norm @ x_norm
    # This u_norm is in normalized control units
    # To convert to physical: u_phys = u_norm * (denormalization factor)
    #
    # Since B_norm[3,0] = (1/max_speed_ms) * B_phys[3,0] = 1/max_speed_ms
    # The control scaling is: u_phys = u_norm * max_speed_ms
    steps_per_meter = 20000.0
    max_speed_ms = max_speed / steps_per_meter  # m/s
    # Convert normalized control to physical acceleration (m/s²)
    acceleration_ms2 = u_normalized * max_speed_ms
    # Convert to steps/s²
    acceleration_steps = acceleration_ms2 * steps_per_meter
    
    # Apply hard limits
    acceleration_steps = max(-max_accel, min(max_accel, acceleration_steps))
    
    # PRIORITY 5: Configurable soft limits
    if rail_length > 0:
        pos_normalized_abs = abs(cart_pos_normalized)
        
        # Soft limit: start reducing acceleration when approaching limits
        if pos_normalized_abs > soft_limit_threshold:
            # Calculate how close we are to the limit
            # 0.0 = at threshold, 1.0 = at limit (normalized position = 1.0)
            distance_from_threshold = pos_normalized_abs - soft_limit_threshold
            distance_to_limit = 1.0 - soft_limit_threshold  # Distance from threshold to limit
            
            # Reduce acceleration progressively: 100% at threshold, 0% at limit
            reduction_factor = 1.0 - (distance_from_threshold / distance_to_limit)
            reduction_factor = max(0.0, min(1.0, reduction_factor))  # Clamp to [0, 1]
            acceleration_steps *= reduction_factor
            
            # Also add a velocity-dependent reduction near limits
            # If moving toward the wall, reduce acceleration more
            if (cart_pos_normalized > 0 and acceleration_steps > 0) or \
               (cart_pos_normalized < 0 and acceleration_steps < 0):
                # Moving toward wall - reduce by additional 50%
                acceleration_steps *= 0.5
    
    # PRIORITY 2: Remove ad-hoc centering (LQI handles this now)
    # No more centering_gain blending - LQI integral action handles steady-state error
    
    # Check if at limits
    at_limit = False
    if position <= limit_left_pos and acceleration_steps < 0:
        acceleration_steps = 0
        at_limit = True
    if position >= limit_right_pos and acceleration_steps > 0:
        acceleration_steps = 0
        at_limit = True
    
    return int(acceleration_steps), at_limit, new_position_integral

