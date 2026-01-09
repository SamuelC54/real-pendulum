"""
Physics calculations for the inverted pendulum system.

Implements coupled cart-pendulum dynamics using full dynamic equations.
"""

import math
import numpy as np
from typing import Tuple


def compute_coupled_dynamics(
    cart_mass: float,
    pendulum_mass: float,
    pendulum_length: float,
    gravity: float,
    damping: float,
    cart_friction: float,
    target_accel_ms2: float,
    cart_pos_m: float,
    cart_vel_ms: float,
    pendulum_angle: float,
    pendulum_velocity: float,
) -> Tuple[float, float]:
    """
    Compute cart and pendulum accelerations using coupled dynamic equations.
    
    Implements coupled cart-pendulum dynamics:
    - Cart: M*x'' = F - m*L*θ''*cos(θ) + m*L*θ'²*sin(θ) - b*x'
    - Pendulum: (m*L²)*θ'' = m*g*L*sin(θ) - m*L*x''*cos(θ) - c*θ'
    
    Convention: θ=0 is DOWN (stable), θ=π is UP (inverted/unstable)
    Adjusted: θ'' = -(g/L)*sin(θ) + (x''/L)*cos(θ) - c*θ'
    
    Args:
        cart_mass: Cart mass (kg)
        pendulum_mass: Pendulum mass (kg)
        pendulum_length: Pendulum length (m)
        gravity: Gravity acceleration (m/s²)
        damping: Angular damping coefficient
        cart_friction: Cart friction coefficient (N·s/m)
        target_accel_ms2: Commanded cart acceleration (m/s²)
        cart_pos_m: Current cart position (m)
        cart_vel_ms: Current cart velocity (m/s)
        pendulum_angle: Current pendulum angle (rad)
        pendulum_velocity: Current pendulum angular velocity (rad/s)
    
    Returns:
        Tuple of (cart_accel_ms2, angular_accel_rad_s2)
    """
    M = cart_mass
    m = pendulum_mass
    L = pendulum_length
    g = gravity
    c = damping
    b = cart_friction
    
    # Current state
    theta = pendulum_angle
    theta_dot = pendulum_velocity
    
    # Trigonometric values
    sin_theta = math.sin(theta)
    cos_theta = math.cos(theta)
    
    # Solve coupled dynamics for x'' and θ''
    # Applied force (from commanded acceleration)
    F = M * target_accel_ms2
    
    # Denominator for solving coupled system
    denom = M + m * sin_theta * sin_theta
    
    # Cart acceleration (solving the coupled system)
    # Note: The m*g*sin(θ)*cos(θ) term comes from the pendulum's weight component
    # that affects the cart. With θ=0 DOWN, this should have the correct sign.
    cart_accel_ms2 = (F + m * L * sin_theta * theta_dot * theta_dot 
                     - m * g * sin_theta * cos_theta 
                     - b * cart_vel_ms) / denom
    
    # Pendulum angular acceleration
    # With convention θ=0 is DOWN (stable), θ=π is UP (unstable)
    # Match the original simplified equation: θ'' = -(g/L)*sin(θ) + (a_cart/L)*cos(θ) - c*θ'
    # The cart acceleration coupling should be positive (not negative)
    angular_accel = (-g * sin_theta + cart_accel_ms2 * cos_theta - c * theta_dot) / L
    
    return cart_accel_ms2, angular_accel


def compute_linearized_matrices(
    cart_mass: float,
    pendulum_mass: float,
    pendulum_length: float,
    gravity: float,
    damping: float,
    cart_friction: float = 0.0,
) -> Tuple[np.ndarray, np.ndarray]:
    """
    Compute linearized state-space matrices (A, B) for LQR control.
    
    Based on CTMS (Control Tutorials for MATLAB and Simulink) standard approach.
    Linearizes the coupled cart-pendulum dynamics around the upright equilibrium (θ = π).
    
    The linearization properly accounts for full coupling between cart and pendulum,
    following the standard derivation from the dynamic equations.
    
    State vector: [θ - π, θ_dot, x, x_dot]
    where θ=0 is DOWN (stable), θ=π is UP (inverted/unstable)
    
    Control input: cart acceleration u (m/s²)
    
    The linearized system is: x' = A*x + B*u
    
    Args:
        cart_mass: Cart mass M (kg)
        pendulum_mass: Pendulum mass m (kg)
        pendulum_length: Pendulum length L (m)
        gravity: Gravity acceleration g (m/s²)
        damping: Angular damping coefficient c
        cart_friction: Cart friction coefficient b (N·s/m), optional
    
    Returns:
        Tuple of (A, B) matrices for state-space representation: x' = A*x + B*u
    """
    M = cart_mass
    m = pendulum_mass
    L = pendulum_length
    g = gravity
    c = damping
    b = cart_friction
    
    # Linearize around θ = π (upright equilibrium)
    # At θ = π: sin(π) = 0, cos(π) = -1
    # For small deviation δ = θ - π: sin(π + δ) ≈ -δ, cos(π + δ) ≈ -1
    
    # From the physics equations (see compute_coupled_dynamics):
    # Cart: M*x'' = F - m*L*θ''*cos(θ) + m*L*θ'²*sin(θ) - b*x'
    # Pendulum: (m*L²)*θ'' = m*g*L*sin(θ) - m*L*x''*cos(θ) - c*θ'
    # Rearranged: θ'' = (-g*sin(θ) + x''*cos(θ) - c*θ') / L
    
    # Linearizing around θ = π + δ:
    # Pendulum: θ'' ≈ (g*δ - x'' - c*θ') / L  [since sin(π+δ)≈-δ, cos(π+δ)≈-1]
    # Cart: M*x'' ≈ F + m*L*θ'' - b*x'  [since cos(π+δ)≈-1, and θ'² terms are 2nd order]
    
    # Solving the coupled system by substituting θ'' into cart equation:
    # M*x'' ≈ F + m*L*(g*δ - x'' - c*θ')/L - b*x'
    # M*x'' ≈ F + m*g*δ - m*x'' - m*c*θ' - b*x'
    # (M + m)*x'' ≈ F + m*g*δ - m*c*θ' - b*x'
    # x'' ≈ (F + m*g*δ - m*c*θ' - b*x') / (M + m)
    
    # Now substitute back into pendulum equation:
    # θ'' ≈ (g*δ - x'' - c*θ') / L
    # θ'' ≈ (g*δ - (F + m*g*δ - m*c*θ' - b*x')/(M+m) - c*θ') / L
    # θ'' ≈ (g*δ*(M+m) - F - m*g*δ + m*c*θ' + b*x' - c*θ'*(M+m)) / (L*(M+m))
    # θ'' ≈ (g*M*δ - F + m*c*θ' + b*x' - c*M*θ' - c*m*θ') / (L*(M+m))
    # θ'' ≈ (g*M*δ - F + b*x' - c*M*θ') / (L*(M+m))
    
    # For control input u = x'' (cart acceleration), we have:
    # The control directly affects x_dot, and couples to θ'' through the dynamics
    
    # Standard CTMS approach: derive A and B matrices from the linearized equations
    # State: x = [θ-π, θ_dot, x, x_dot]'
    
    # A matrix: describes how states evolve
    # d(θ-π)/dt = θ_dot
    # d(θ_dot)/dt = θ'' ≈ (g*M*δ - u + b*x_dot - c*M*θ_dot) / (L*(M+m))
    #                = (g*M/(L*(M+m)))*(θ-π) - (c*M/(L*(M+m)))*θ_dot + (b/(L*(M+m)))*x_dot - (1/(L*(M+m)))*u
    # dx/dt = x_dot
    # dx_dot/dt = x'' ≈ (u + m*g*δ - m*c*θ_dot - b*x_dot) / (M+m)
    #                = (m*g/(M+m))*(θ-π) - (m*c/(M+m))*θ_dot - (b/(M+m))*x_dot + (1/(M+m))*u
    
    # However, since our control is acceleration (u = x''), we need to account for this:
    # The B matrix should reflect how u directly affects the system
    
    # Simplified approach matching CTMS standard (accounting for coupling):
    # A[1,0] = g*M/(L*(M+m))  (pendulum angle affects angular acceleration)
    # A[1,1] = -c*M/(L*(M+m))  (angular velocity damping)
    # A[1,3] = b/(L*(M+m))     (cart velocity affects pendulum)
    # A[3,0] = m*g/(M+m)       (pendulum angle affects cart acceleration)
    # A[3,1] = -m*c/(M+m)      (angular velocity affects cart)
    # A[3,3] = -b/(M+m)        (cart velocity friction)
    
    # But wait - our control is acceleration, not force. So u = x'' directly.
    # This means B[3,0] = 1 (direct effect on cart velocity)
    # And B[1,0] accounts for coupling from cart acceleration to pendulum
    
    # Using the standard CTMS derivation with proper coupling:
    denom = M + m  # Denominator for coupled system
    
    # A matrix: state is [θ-π, θ_dot, x, x_dot]
    A = np.array([
        [0, 1, 0, 0],                                    # d(θ-π)/dt = θ_dot
        [g * M / (L * denom), -c * M / (L * denom), 0, b / (L * denom)],  # d(θ_dot)/dt
        [0, 0, 0, 1],                                    # dx/dt = x_dot
        [m * g / denom, -m * c / denom, 0, -b / denom]  # dx_dot/dt
    ])
    
    # B matrix: control input is cart acceleration u (m/s²)
    # u directly affects x_dot, and couples to θ_dot through the dynamics
    # From the linearized equations:
    # θ'' includes -u/(L*(M+m)) term
    # x'' = u (by definition, since u is acceleration)
    B = np.array([
        [0],                    # θ-π doesn't directly depend on control
        [-1 / (L * denom)],     # Pendulum: θ_dot couples to cart acceleration
        [0],                    # Position doesn't directly depend on control
        [1]                     # Cart: x_dot directly responds to acceleration
    ])
    
    return A, B

