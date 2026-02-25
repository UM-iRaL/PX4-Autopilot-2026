"""
Trajectory generation and management modules.

This package provides trajectory implementations for drone flight control.

Main classes:
- TrajectoryManager: Orchestrates trajectory lifecycle (start, stop, get_setpoint)
- WaitTrajectory: Time-scaled path following with dynamic feedback

Base classes (for extending):
- BaseTrajectory: Abstract base with shared utilities
- StatelessTrajectory: For pure time-based trajectories
- StatefulTrajectory: For trajectories with initialization/cleanup

Trajectory implementations are organized in subdirectories:
- stateless/: Roll, pitch, yaw sweeps, circular motion, combined rotation
- stateful/: Takeoff, landing, hand tracking, wait trajectory wrapper
"""

from .wait_trajectory import WaitTrajectory
from .trajectory_manager import TrajectoryManager
from .base import BaseTrajectory, StatelessTrajectory, StatefulTrajectory

# Stateless trajectories
from .stateless import (
    AngularSweepTrajectory,
    CircularMotionTrajectory,
    CombinedRotationTrajectory,
)

# Stateful trajectories
from .stateful import (
    TakeoffTrajectory,
    LandingTrajectory,
    HandTrackingTrajectory,
    WaitTrajectoryWrapper,
)

__all__ = [
    # Main classes (backward compatible)
    'WaitTrajectory',
    'TrajectoryManager',

    # Base classes for extending
    'BaseTrajectory',
    'StatelessTrajectory',
    'StatefulTrajectory',

    # Stateless trajectory implementations
    'AngularSweepTrajectory',
    'CircularMotionTrajectory',
    'CombinedRotationTrajectory',

    # Stateful trajectory implementations
    'TakeoffTrajectory',
    'LandingTrajectory',
    'HandTrackingTrajectory',
    'WaitTrajectoryWrapper',
]
