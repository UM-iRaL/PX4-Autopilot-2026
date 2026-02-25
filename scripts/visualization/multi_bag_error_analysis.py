#!/usr/bin/env python3
"""
Multi-Bag Error Distribution Analysis for Academic Publications.

This script analyzes multiple ROS bag files from repeated flight experiments
and generates publication-quality visualizations of error distributions.

Visualization options:
    - Violin plots: Show full error distribution across runs (position, rotation, quaternion)
    - Error bars: Mean with standard deviation or confidence intervals
    - Error envelope: Nominal trajectory with min/max error bounds
    - Rotation error (e_R): SO(3) rotation error from geometric control (Lee et al.)
    - Quaternion error: Component-wise quaternion tracking error

Plot types (--plot-type):
    violin      - Position error violin plots
    error_bars  - Time series with mean +/- std or confidence interval
    envelope    - 2D/3D trajectory with min/max error bounds
    rotation    - SO(3) rotation error (e_R) violin and time series
    quaternion  - Quaternion component error violin and time series
    summary     - Statistics table
    all         - Generate all plots (default)

Usage:
    python3 scripts/visualization/multi_bag_error_analysis.py ~/data/wind_0_pitch/*.bag --inspect     --start-altitude 1.0     --end-altitude 0.95 --run-analysis
    python multi_bag_error_analysis.py bag1.bag bag2.bag bag3.bag [options]
    python multi_bag_error_analysis.py *.bag --plot-type violin --output figures/
    python multi_bag_error_analysis.py *.bag --plot-type rotation quaternion
    python multi_bag_error_analysis.py *.bag --plot-type envelope --format png --dpi 600

Author: IRAL Lab
"""

import argparse
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple, Union
import numpy as np
from scipy import stats
from scipy.interpolate import interp1d

# Plotting imports
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving figures
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MaxNLocator
from matplotlib.patches import Patch
from matplotlib.collections import PolyCollection
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ROS bag reading
import rosbag


# ==============================================================================
# Publication-Quality Plot Configuration
# ==============================================================================

# IEEE/Conference standard: column width ~3.5in, full width ~7in
COLUMN_WIDTH_INCHES = 3.5
FULL_WIDTH_INCHES = 7.16
GOLDEN_RATIO = 1.618

# Colorblind-friendly palette (Wong, 2011 + Tol's vibrant)
# Extended for multiple runs comparison
# Consistent axis colors: x/roll = blue, y/pitch = orange, z/yaw = green
COLORS = {
    # Primary colors
    'blue': '#0072B2',
    'orange': '#D55E00',
    'green': '#009E73',
    'yellow': '#F0E442',
    'sky_blue': '#56B4E9',
    'vermillion': '#E69F00',
    'purple': '#CC79A7',
    'black': '#000000',
    # Axis colors (consistent across position, rotation, orientation)
    'x': '#0072B2',       # Blue for x / roll
    'y': '#D55E00',       # Orange for y / pitch
    'z': '#009E73',       # Green for z / yaw
    'norm': '#CC79A7',    # Purple for norms / magnitudes
    # Semantic colors
    'nominal': '#0072B2',
    'error_fill': '#56B4E9',
    'error_line': '#CC79A7',
    'envelope_fill': '#009E73',
    'mean': '#000000',
    'median': '#CC79A7',
    'grid': '#CCCCCC',
}

# Palette for multiple runs (8 distinct colors)
RUN_COLORS = [
    '#0072B2',  # Blue
    '#D55E00',  # Vermillion
    '#009E73',  # Bluish green
    '#E69F00',  # Orange
    '#56B4E9',  # Sky blue
    '#CC79A7',  # Reddish purple
    '#F0E442',  # Yellow
    '#000000',  # Black
]

# Violin/box plot colors
VIOLIN_COLORS = {
    'face': '#56B4E9',      # Sky blue (light)
    'edge': '#0072B2',      # Blue (dark)
    'median': '#D55E00',    # Vermillion
    'mean': '#009E73',      # Green
    'whisker': '#000000',   # Black
}


def setup_publication_style():
    """Configure matplotlib for publication-quality figures."""
    plt.rcParams.update({
        # Font settings
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif', 'Computer Modern Roman'],
        'font.size': 9,
        'axes.titlesize': 10,
        'axes.labelsize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 8,

        # LaTeX rendering
        'text.usetex': False,
        'mathtext.fontset': 'cm',

        # Figure settings
        'figure.dpi': 150,
        'savefig.dpi': 300,
        'savefig.format': 'pdf',
        'savefig.bbox': 'tight',
        'savefig.pad_inches': 0.02,

        # Axes settings
        'axes.linewidth': 0.6,
        'axes.grid': True,
        'axes.axisbelow': True,
        'axes.spines.top': False,
        'axes.spines.right': False,

        # Grid settings
        'grid.linewidth': 0.4,
        'grid.alpha': 0.5,
        'grid.linestyle': '-',

        # Tick settings
        'xtick.major.width': 0.6,
        'ytick.major.width': 0.6,
        'xtick.minor.width': 0.4,
        'ytick.minor.width': 0.4,
        'xtick.major.size': 3,
        'ytick.major.size': 3,
        'xtick.minor.size': 1.5,
        'ytick.minor.size': 1.5,
        'xtick.direction': 'in',
        'ytick.direction': 'in',

        # Legend settings
        'legend.framealpha': 0.9,
        'legend.edgecolor': 'none',
        'legend.fancybox': False,
        'legend.frameon': True,

        # Line settings
        'lines.linewidth': 1.2,
        'lines.markersize': 4,
    })


# ==============================================================================
# Data Classes
# ==============================================================================

@dataclass
class FlightData:
    """Container for flight data from a single bag."""
    bag_path: str
    name: str = ""
    # Timestamps (normalized to start at 0)
    timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    # Position (actual)
    x: np.ndarray = field(default_factory=lambda: np.array([]))
    y: np.ndarray = field(default_factory=lambda: np.array([]))
    z: np.ndarray = field(default_factory=lambda: np.array([]))
    # Position setpoint
    x_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    y_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    z_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    # Velocity (actual)
    vx: np.ndarray = field(default_factory=lambda: np.array([]))
    vy: np.ndarray = field(default_factory=lambda: np.array([]))
    vz: np.ndarray = field(default_factory=lambda: np.array([]))
    # Velocity setpoint
    vx_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    vy_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    vz_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    # Quaternion (actual)
    qx: np.ndarray = field(default_factory=lambda: np.array([]))
    qy: np.ndarray = field(default_factory=lambda: np.array([]))
    qz: np.ndarray = field(default_factory=lambda: np.array([]))
    qw: np.ndarray = field(default_factory=lambda: np.array([]))
    # Quaternion setpoint
    qx_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    qy_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    qz_sp: np.ndarray = field(default_factory=lambda: np.array([]))
    qw_sp: np.ndarray = field(default_factory=lambda: np.array([]))


@dataclass
class ErrorStatistics:
    """Container for error statistics across multiple runs."""
    # Common time grid
    time_grid: np.ndarray = field(default_factory=lambda: np.array([]))
    # Per-axis errors at each time point (shape: n_runs x n_time_points)
    ex_all: np.ndarray = field(default_factory=lambda: np.array([]))
    ey_all: np.ndarray = field(default_factory=lambda: np.array([]))
    ez_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Euclidean position error
    e_norm_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Velocity errors
    evx_all: np.ndarray = field(default_factory=lambda: np.array([]))
    evy_all: np.ndarray = field(default_factory=lambda: np.array([]))
    evz_all: np.ndarray = field(default_factory=lambda: np.array([]))
    ev_norm_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Rotation error (SO(3)) - norm of error vector
    e_R_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Rotation error components (from vee map)
    e_R_x_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_R_y_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_R_z_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Quaternion errors (actual - setpoint, with sign correction)
    e_qx_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_qy_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_qz_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_qw_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Euler angle errors (degrees)
    e_roll_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_pitch_all: np.ndarray = field(default_factory=lambda: np.array([]))
    e_yaw_all: np.ndarray = field(default_factory=lambda: np.array([]))
    # Reference trajectory (from first bag or specified)
    x_ref: np.ndarray = field(default_factory=lambda: np.array([]))
    y_ref: np.ndarray = field(default_factory=lambda: np.array([]))
    z_ref: np.ndarray = field(default_factory=lambda: np.array([]))


# ==============================================================================
# Quaternion Utilities
# ==============================================================================

def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Convert quaternion to 3x3 rotation matrix."""
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    if norm < 1e-10:
        return np.eye(3)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R


def vee_map(S: np.ndarray) -> np.ndarray:
    """Extract vector from skew-symmetric matrix."""
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def compute_rotation_error(R: np.ndarray, R_d: np.ndarray) -> np.ndarray:
    """Compute SO(3) rotation error vector."""
    error_matrix = R_d.T @ R - R.T @ R_d
    return 0.5 * vee_map(error_matrix)


def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """Convert quaternion to Euler angles (roll, pitch, yaw) in radians."""
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def normalize_quaternion_continuous(qx: np.ndarray, qy: np.ndarray,
                                     qz: np.ndarray, qw: np.ndarray) -> Tuple[np.ndarray, ...]:
    """Normalize quaternions for temporal continuity."""
    if len(qw) == 0:
        return qx, qy, qz, qw

    qx_out, qy_out, qz_out, qw_out = qx.copy(), qy.copy(), qz.copy(), qw.copy()

    if qw_out[0] < 0:
        qx_out[0], qy_out[0], qz_out[0], qw_out[0] = -qx_out[0], -qy_out[0], -qz_out[0], -qw_out[0]

    for i in range(1, len(qw)):
        dot = (qx_out[i] * qx_out[i-1] + qy_out[i] * qy_out[i-1] +
               qz_out[i] * qz_out[i-1] + qw_out[i] * qw_out[i-1])
        if dot < 0:
            qx_out[i], qy_out[i], qz_out[i], qw_out[i] = -qx_out[i], -qy_out[i], -qz_out[i], -qw_out[i]

    return qx_out, qy_out, qz_out, qw_out


# ==============================================================================
# Bag Reader
# ==============================================================================

class MultiBagReader:
    """Read and process multiple ROS bags for error analysis."""

    TOPIC_ODOMETRY = '/mavros/odometry/in'
    TOPIC_POSITION_SETPOINT = '/mavros/setpoint_raw/local'
    TOPIC_ATTITUDE_SETPOINT = '/mavros/setpoint_raw/attitude'

    def __init__(self, bag_paths: List[str], names: Optional[List[str]] = None):
        """
        Initialize multi-bag reader.

        Args:
            bag_paths: List of paths to ROS bag files
            names: Optional list of names for each run (for legends)
        """
        self.bag_paths = bag_paths
        self.names = names if names else [f"Run {i+1}" for i in range(len(bag_paths))]
        self.flight_data: List[FlightData] = []

    def read_all(self, start_time: Optional[float] = None,
                 end_time: Optional[float] = None,
                 per_bag_times: Optional[List[Tuple[float, float]]] = None) -> None:
        """
        Read all bag files.

        Args:
            start_time: Global start time for all bags (ignored if per_bag_times provided)
            end_time: Global end time for all bags (ignored if per_bag_times provided)
            per_bag_times: List of (start, end) tuples, one per bag file
        """
        for i, bag_path in enumerate(self.bag_paths):
            print(f"Reading bag {i+1}/{len(self.bag_paths)}: {bag_path}")

            if per_bag_times and i < len(per_bag_times):
                bag_start, bag_end = per_bag_times[i]
            else:
                bag_start, bag_end = start_time, end_time

            data = self._read_single_bag(bag_path, self.names[i], bag_start, bag_end)
            self.flight_data.append(data)

    def _read_single_bag(self, bag_path: str, name: str,
                         start_time: Optional[float],
                         end_time: Optional[float]) -> FlightData:
        """Read a single bag file."""
        data = FlightData(bag_path=bag_path, name=name)

        odom_data = {'t': [], 'x': [], 'y': [], 'z': [],
                     'vx': [], 'vy': [], 'vz': [],
                     'qx': [], 'qy': [], 'qz': [], 'qw': []}
        pos_sp_data = {'t': [], 'x': [], 'y': [], 'z': [],
                       'vx': [], 'vy': [], 'vz': []}
        att_sp_data = {'t': [], 'qx': [], 'qy': [], 'qz': [], 'qw': []}

        with rosbag.Bag(bag_path, 'r') as bag:
            bag_start = bag.get_start_time()
            abs_start = bag_start + (start_time if start_time else 0)
            abs_end = bag.get_end_time() if end_time is None else bag_start + end_time

            for topic, msg, t in bag.read_messages():
                t_sec = t.to_sec()
                if t_sec < abs_start or t_sec > abs_end:
                    continue

                rel_time = t_sec - abs_start

                if topic == self.TOPIC_ODOMETRY:
                    odom_data['t'].append(rel_time)
                    odom_data['x'].append(msg.pose.pose.position.x)
                    odom_data['y'].append(msg.pose.pose.position.y)
                    odom_data['z'].append(msg.pose.pose.position.z)
                    odom_data['vx'].append(msg.twist.twist.linear.x)
                    odom_data['vy'].append(msg.twist.twist.linear.y)
                    odom_data['vz'].append(msg.twist.twist.linear.z)
                    odom_data['qx'].append(msg.pose.pose.orientation.x)
                    odom_data['qy'].append(msg.pose.pose.orientation.y)
                    odom_data['qz'].append(msg.pose.pose.orientation.z)
                    odom_data['qw'].append(msg.pose.pose.orientation.w)

                elif topic == self.TOPIC_POSITION_SETPOINT:
                    pos_sp_data['t'].append(rel_time)
                    pos_sp_data['x'].append(msg.position.x)
                    pos_sp_data['y'].append(msg.position.y)
                    pos_sp_data['z'].append(msg.position.z)
                    pos_sp_data['vx'].append(msg.velocity.x)
                    pos_sp_data['vy'].append(msg.velocity.y)
                    pos_sp_data['vz'].append(msg.velocity.z)

                elif topic == self.TOPIC_ATTITUDE_SETPOINT:
                    att_sp_data['t'].append(rel_time)
                    att_sp_data['qx'].append(msg.orientation.x)
                    att_sp_data['qy'].append(msg.orientation.y)
                    att_sp_data['qz'].append(msg.orientation.z)
                    att_sp_data['qw'].append(msg.orientation.w)

        # Convert to arrays
        data.timestamps = np.array(odom_data['t'])
        data.x = np.array(odom_data['x'])
        data.y = np.array(odom_data['y'])
        data.z = np.array(odom_data['z'])
        data.vx = np.array(odom_data['vx'])
        data.vy = np.array(odom_data['vy'])
        data.vz = np.array(odom_data['vz'])
        data.qx = np.array(odom_data['qx'])
        data.qy = np.array(odom_data['qy'])
        data.qz = np.array(odom_data['qz'])
        data.qw = np.array(odom_data['qw'])

        # Interpolate setpoints to odometry timestamps
        if len(pos_sp_data['t']) > 1 and len(data.timestamps) > 0:
            sp_t = np.array(pos_sp_data['t'])
            data.x_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['x']))
            data.y_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['y']))
            data.z_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['z']))
            data.vx_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['vx']))
            data.vy_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['vy']))
            data.vz_sp = np.interp(data.timestamps, sp_t, np.array(pos_sp_data['vz']))

        if len(att_sp_data['t']) > 1 and len(data.timestamps) > 0:
            sp_t = np.array(att_sp_data['t'])
            data.qx_sp = np.interp(data.timestamps, sp_t, np.array(att_sp_data['qx']))
            data.qy_sp = np.interp(data.timestamps, sp_t, np.array(att_sp_data['qy']))
            data.qz_sp = np.interp(data.timestamps, sp_t, np.array(att_sp_data['qz']))
            data.qw_sp = np.interp(data.timestamps, sp_t, np.array(att_sp_data['qw']))

        print(f"  Loaded {len(data.timestamps)} samples")
        return data

    def compute_error_statistics(self, time_resolution: float = 0.01) -> ErrorStatistics:
        """
        Compute error statistics across all runs.

        Args:
            time_resolution: Time step for the common time grid (seconds)

        Returns:
            ErrorStatistics with all error data aligned to common time grid
        """
        if not self.flight_data:
            raise ValueError("No flight data loaded. Call read_all() first.")

        # Find common time range
        t_min = max(fd.timestamps[0] for fd in self.flight_data if len(fd.timestamps) > 0)
        t_max = min(fd.timestamps[-1] for fd in self.flight_data if len(fd.timestamps) > 0)

        # Create common time grid
        time_grid = np.arange(t_min, t_max, time_resolution)
        n_points = len(time_grid)
        n_runs = len(self.flight_data)

        # Initialize error arrays
        stats = ErrorStatistics()
        stats.time_grid = time_grid
        stats.ex_all = np.zeros((n_runs, n_points))
        stats.ey_all = np.zeros((n_runs, n_points))
        stats.ez_all = np.zeros((n_runs, n_points))
        stats.e_norm_all = np.zeros((n_runs, n_points))
        stats.evx_all = np.zeros((n_runs, n_points))
        stats.evy_all = np.zeros((n_runs, n_points))
        stats.evz_all = np.zeros((n_runs, n_points))
        stats.ev_norm_all = np.zeros((n_runs, n_points))
        stats.e_R_all = np.zeros((n_runs, n_points))
        stats.e_R_x_all = np.zeros((n_runs, n_points))
        stats.e_R_y_all = np.zeros((n_runs, n_points))
        stats.e_R_z_all = np.zeros((n_runs, n_points))
        stats.e_qx_all = np.zeros((n_runs, n_points))
        stats.e_qy_all = np.zeros((n_runs, n_points))
        stats.e_qz_all = np.zeros((n_runs, n_points))
        stats.e_qw_all = np.zeros((n_runs, n_points))
        stats.e_roll_all = np.zeros((n_runs, n_points))
        stats.e_pitch_all = np.zeros((n_runs, n_points))
        stats.e_yaw_all = np.zeros((n_runs, n_points))

        # Use first run's setpoint as reference trajectory
        ref_data = self.flight_data[0]
        stats.x_ref = np.interp(time_grid, ref_data.timestamps, ref_data.x_sp)
        stats.y_ref = np.interp(time_grid, ref_data.timestamps, ref_data.y_sp)
        stats.z_ref = np.interp(time_grid, ref_data.timestamps, ref_data.z_sp)

        # Compute errors for each run
        for i, fd in enumerate(self.flight_data):
            # Interpolate actual positions to common time grid
            x_interp = np.interp(time_grid, fd.timestamps, fd.x)
            y_interp = np.interp(time_grid, fd.timestamps, fd.y)
            z_interp = np.interp(time_grid, fd.timestamps, fd.z)

            # Interpolate setpoints
            x_sp_interp = np.interp(time_grid, fd.timestamps, fd.x_sp)
            y_sp_interp = np.interp(time_grid, fd.timestamps, fd.y_sp)
            z_sp_interp = np.interp(time_grid, fd.timestamps, fd.z_sp)

            # Position errors
            stats.ex_all[i] = x_interp - x_sp_interp
            stats.ey_all[i] = y_interp - y_sp_interp
            stats.ez_all[i] = z_interp - z_sp_interp
            stats.e_norm_all[i] = np.sqrt(stats.ex_all[i]**2 +
                                           stats.ey_all[i]**2 +
                                           stats.ez_all[i]**2)

            # Velocity errors (if available)
            if len(fd.vx) > 0 and len(fd.vx_sp) > 0:
                vx_interp = np.interp(time_grid, fd.timestamps, fd.vx)
                vy_interp = np.interp(time_grid, fd.timestamps, fd.vy)
                vz_interp = np.interp(time_grid, fd.timestamps, fd.vz)
                vx_sp_interp = np.interp(time_grid, fd.timestamps, fd.vx_sp)
                vy_sp_interp = np.interp(time_grid, fd.timestamps, fd.vy_sp)
                vz_sp_interp = np.interp(time_grid, fd.timestamps, fd.vz_sp)

                stats.evx_all[i] = vx_interp - vx_sp_interp
                stats.evy_all[i] = vy_interp - vy_sp_interp
                stats.evz_all[i] = vz_interp - vz_sp_interp
                stats.ev_norm_all[i] = np.sqrt(stats.evx_all[i]**2 +
                                                stats.evy_all[i]**2 +
                                                stats.evz_all[i]**2)

            # Rotation error (if quaternion data available)
            if len(fd.qx) > 0 and len(fd.qx_sp) > 0:
                # Interpolate quaternions
                qx_interp = np.interp(time_grid, fd.timestamps, fd.qx)
                qy_interp = np.interp(time_grid, fd.timestamps, fd.qy)
                qz_interp = np.interp(time_grid, fd.timestamps, fd.qz)
                qw_interp = np.interp(time_grid, fd.timestamps, fd.qw)
                qx_sp_interp = np.interp(time_grid, fd.timestamps, fd.qx_sp)
                qy_sp_interp = np.interp(time_grid, fd.timestamps, fd.qy_sp)
                qz_sp_interp = np.interp(time_grid, fd.timestamps, fd.qz_sp)
                qw_sp_interp = np.interp(time_grid, fd.timestamps, fd.qw_sp)

                # Normalize quaternions for continuity
                qx_interp, qy_interp, qz_interp, qw_interp = normalize_quaternion_continuous(
                    qx_interp, qy_interp, qz_interp, qw_interp)
                qx_sp_interp, qy_sp_interp, qz_sp_interp, qw_sp_interp = normalize_quaternion_continuous(
                    qx_sp_interp, qy_sp_interp, qz_sp_interp, qw_sp_interp)

                for j in range(n_points):
                    R = quaternion_to_rotation_matrix(qx_interp[j], qy_interp[j],
                                                      qz_interp[j], qw_interp[j])
                    R_d = quaternion_to_rotation_matrix(qx_sp_interp[j], qy_sp_interp[j],
                                                        qz_sp_interp[j], qw_sp_interp[j])
                    e_R = compute_rotation_error(R, R_d)
                    stats.e_R_all[i, j] = np.linalg.norm(e_R)
                    stats.e_R_x_all[i, j] = e_R[0]
                    stats.e_R_y_all[i, j] = e_R[1]
                    stats.e_R_z_all[i, j] = e_R[2]

                # Quaternion error: compute error quaternion q_e = q_sp^{-1} * q
                # For small errors, the vector part approximates half the rotation error
                # We use a simpler metric: component-wise difference with sign alignment
                for j in range(n_points):
                    # Align signs: if dot product is negative, flip actual quaternion
                    dot = (qx_interp[j] * qx_sp_interp[j] + qy_interp[j] * qy_sp_interp[j] +
                           qz_interp[j] * qz_sp_interp[j] + qw_interp[j] * qw_sp_interp[j])
                    sign = 1.0 if dot >= 0 else -1.0

                    stats.e_qx_all[i, j] = sign * qx_interp[j] - qx_sp_interp[j]
                    stats.e_qy_all[i, j] = sign * qy_interp[j] - qy_sp_interp[j]
                    stats.e_qz_all[i, j] = sign * qz_interp[j] - qz_sp_interp[j]
                    stats.e_qw_all[i, j] = sign * qw_interp[j] - qw_sp_interp[j]

                # Euler angle errors (convert quaternions to euler, then difference in degrees)
                for j in range(n_points):
                    roll_act, pitch_act, yaw_act = quaternion_to_euler(
                        qx_interp[j], qy_interp[j], qz_interp[j], qw_interp[j])
                    roll_sp, pitch_sp, yaw_sp = quaternion_to_euler(
                        qx_sp_interp[j], qy_sp_interp[j], qz_sp_interp[j], qw_sp_interp[j])
                    # Wrap angle differences to [-pi, pi] then convert to degrees
                    stats.e_roll_all[i, j] = np.degrees(np.arctan2(
                        np.sin(roll_act - roll_sp), np.cos(roll_act - roll_sp)))
                    stats.e_pitch_all[i, j] = np.degrees(np.arctan2(
                        np.sin(pitch_act - pitch_sp), np.cos(pitch_act - pitch_sp)))
                    stats.e_yaw_all[i, j] = np.degrees(np.arctan2(
                        np.sin(yaw_act - yaw_sp), np.cos(yaw_act - yaw_sp)))

        return stats


# ==============================================================================
# Error Distribution Plotter
# ==============================================================================

class ErrorDistributionPlotter:
    """Generate publication-quality error distribution plots."""

    def __init__(self, reader: MultiBagReader, output_dir: str = '.',
                 dpi: int = 300, format: str = 'pdf'):
        """
        Initialize the plotter.

        Args:
            reader: MultiBagReader with loaded data
            output_dir: Directory to save figures
            dpi: Resolution for raster formats
            format: Output format (pdf, png, eps, svg)
        """
        self.reader = reader
        self.output_dir = output_dir
        self.dpi = dpi
        self.format = format
        self.stats: Optional[ErrorStatistics] = None

        os.makedirs(output_dir, exist_ok=True)
        setup_publication_style()

    def compute_statistics(self, time_resolution: float = 0.01) -> None:
        """Compute error statistics from loaded data."""
        self.stats = self.reader.compute_error_statistics(time_resolution)

    def _save_figure(self, fig: plt.Figure, name: str) -> str:
        """Save figure and return path."""
        path = os.path.join(self.output_dir, f"{name}.{self.format}")
        fig.savefig(path, dpi=self.dpi, format=self.format)
        plt.close(fig)
        print(f"Saved: {path}")
        return path

    def _add_minor_ticks(self, ax: plt.Axes) -> None:
        """Add minor ticks to axis."""
        ax.xaxis.set_minor_locator(AutoMinorLocator(2))
        ax.yaxis.set_minor_locator(AutoMinorLocator(2))

    # ==========================================================================
    # Violin Plots
    # ==========================================================================

    def plot_violin_position_error(self, show_individual: bool = True) -> str:
        """
        Create violin plots showing position error distribution.

        Args:
            show_individual: If True, overlay individual data points

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(2, 2, figsize=(FULL_WIDTH_INCHES, 5.0))

        # Flatten errors for violin plots (across all time points)
        error_data = [
            ('$e_x$', self.stats.ex_all.flatten(), COLORS['x']),
            ('$e_y$', self.stats.ey_all.flatten(), COLORS['y']),
            ('$e_z$', self.stats.ez_all.flatten(), COLORS['z']),
            (r'$\|e\|$', self.stats.e_norm_all.flatten(), COLORS['norm']),
        ]

        # Compute shared y-axis limits for per-axis violins (ex, ey, ez)
        axis_flat = [self.stats.ex_all.flatten(), self.stats.ey_all.flatten(),
                     self.stats.ez_all.flatten()]
        y_min = min(np.min(d) for d in axis_flat)
        y_max = max(np.max(d) for d in axis_flat)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for idx, (ax, (label, data, color)) in enumerate(zip(axes.flatten(), error_data)):
            # Create violin plot (no mean line)
            parts = ax.violinplot([data], positions=[0], showmeans=False,
                                  showmedians=True, showextrema=False)

            # Style the violin
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.3)

            parts['cmedians'].set_color(VIOLIN_COLORS['median'])
            parts['cmedians'].set_linewidth(1.5)

            # Add box plot overlay for quartiles with whiskers (1.5x IQR)
            bp = ax.boxplot([data], positions=[0], widths=0.15,
                           patch_artist=True, showfliers=False, whis=1.5)
            bp['boxes'][0].set_facecolor('white')
            bp['boxes'][0].set_edgecolor(color)
            bp['boxes'][0].set_alpha(0.8)
            bp['medians'][0].set_color(VIOLIN_COLORS['median'])
            bp['whiskers'][0].set_color(color)
            bp['whiskers'][1].set_color(color)
            bp['caps'][0].set_color(color)
            bp['caps'][1].set_color(color)

            # RMSE annotation as box above the violin
            rmse = np.sqrt(np.mean(data**2))
            ax.annotate(f'RMSE: {rmse:.4f} m',
                       xy=(0, 1.02), xycoords=('data', 'axes fraction'),
                       ha='center', va='bottom', fontsize=7,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor=color, alpha=0.9))

            ax.set_ylabel(f'{label} (m)')
            # Share y-axis limits for per-axis violins (not the norm)
            if idx < 3:
                ax.set_ylim(shared_ylim)
            ax.set_xticks([])
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout()

        return self._save_figure(fig, 'violin_position_error')

    def plot_violin_per_run(self) -> str:
        """
        Create violin plots comparing error distribution across runs.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        n_runs = len(self.reader.flight_data)
        fig, ax = plt.subplots(figsize=(FULL_WIDTH_INCHES, 3.5))

        # Collect position error norm for each run
        positions = np.arange(n_runs)
        data = [self.stats.e_norm_all[i] for i in range(n_runs)]

        # Create violin plot (no mean line)
        parts = ax.violinplot(data, positions=positions, showmeans=False,
                              showmedians=True, showextrema=False)

        # Color each violin differently
        for i, pc in enumerate(parts['bodies']):
            color = RUN_COLORS[i % len(RUN_COLORS)]
            pc.set_facecolor(color)
            pc.set_edgecolor(color)
            pc.set_alpha(0.4)

        parts['cmedians'].set_color(VIOLIN_COLORS['median'])
        parts['cmedians'].set_linewidth(1.5)

        # Labels
        ax.set_xticks(positions)
        ax.set_xticklabels([fd.name for fd in self.reader.flight_data], rotation=45, ha='right')
        ax.set_ylabel(r'$\|e\|$ (m)')
        ax.set_xlabel('Run')

        # Add RMSE annotation above each violin
        rmse_all = [np.sqrt(np.mean(d**2)) for d in data]
        for i, rmse in enumerate(rmse_all):
            color = RUN_COLORS[i % len(RUN_COLORS)]
            ax.annotate(f'RMSE: {rmse:.4f}',
                       xy=(i, 1.02), xycoords=('data', 'axes fraction'),
                       ha='center', va='bottom', fontsize=6,
                       bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                 edgecolor=color, alpha=0.9))

        self._add_minor_ticks(ax)
        ax.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout()

        return self._save_figure(fig, 'violin_per_run')

    # ==========================================================================
    # Error Bar Plots
    # ==========================================================================

    def plot_error_bars_time_series(self, ci: float = 0.95,
                                     error_type: str = 'std') -> str:
        """
        Plot time series with error bars (mean + std or CI).

        Args:
            ci: Confidence interval level (for error_type='ci')
            error_type: 'std' for standard deviation, 'ci' for confidence interval

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(4, 1, figsize=(FULL_WIDTH_INCHES, 6.0), sharex=True)

        time = self.stats.time_grid
        errors = [
            ('$e_x$', self.stats.ex_all, COLORS['x']),
            ('$e_y$', self.stats.ey_all, COLORS['y']),
            ('$e_z$', self.stats.ez_all, COLORS['z']),
            (r'$\|e\|$', self.stats.e_norm_all, COLORS['norm']),
        ]

        for ax, (label, data, color) in zip(axes, errors):
            mean = np.mean(data, axis=0)
            std = np.std(data, axis=0)

            if error_type == 'ci':
                n = data.shape[0]
                se = std / np.sqrt(n)
                t_val = stats.t.ppf((1 + ci) / 2, n - 1)
                error = t_val * se
            else:
                error = std

            # Plot mean line
            ax.plot(time, mean, color=color, linewidth=1.2, label='Mean')

            # Plot error band
            ax.fill_between(time, mean - error, mean + error,
                           color=color, alpha=0.25,
                           label=f'$\\pm${error_type.upper()}' if error_type == 'std' else f'{int(ci*100)}% CI')

            ax.set_ylabel(f'{label} (m)')
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

            if ax == axes[0]:
                ax.legend(loc='upper right', fontsize=7, framealpha=0.9)

        axes[-1].set_xlabel('Time (s)')

        # Add overall statistics
        rmse = np.sqrt(np.mean(self.stats.e_norm_all**2))
        max_err = np.max(self.stats.e_norm_all)
        axes[-1].annotate(f'Overall RMSE: {rmse:.4f} m, Max: {max_err:.4f} m',
                         xy=(0.98, 0.95), xycoords='axes fraction',
                         ha='right', va='top', fontsize=7,
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        fig.tight_layout()

        return self._save_figure(fig, f'error_bars_{error_type}')

    def plot_error_bars_summary(self) -> str:
        """
        Create summary bar chart with error bars for each axis.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, ax = plt.subplots(figsize=(COLUMN_WIDTH_INCHES, 2.5))

        # Calculate RMSE for each axis across all runs
        labels = ['$e_x$', '$e_y$', '$e_z$', r'$\|e\|$']
        errors_data = [
            self.stats.ex_all.flatten(),
            self.stats.ey_all.flatten(),
            self.stats.ez_all.flatten(),
            self.stats.e_norm_all.flatten(),
        ]

        means = [np.mean(np.abs(d)) for d in errors_data]
        stds = [np.std(np.abs(d)) for d in errors_data]
        colors = [COLORS['x'], COLORS['y'], COLORS['z'], COLORS['norm']]

        x = np.arange(len(labels))
        bars = ax.bar(x, means, yerr=stds, capsize=4, color=colors, alpha=0.8,
                     edgecolor=[c for c in colors], linewidth=1.0)

        ax.set_xticks(x)
        ax.set_xticklabels(labels)
        ax.set_ylabel('Mean Absolute Error (m)')
        self._add_minor_ticks(ax)
        ax.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout()

        return self._save_figure(fig, 'error_bars_summary')

    # ==========================================================================
    # Error Envelope Plots
    # ==========================================================================

    def plot_error_envelope_2d(self, plane: str = 'xy') -> str:
        """
        Plot 2D trajectory with error envelope (min/max bounds).

        Args:
            plane: Which plane to plot ('xy', 'xz', 'yz')

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, ax = plt.subplots(figsize=(COLUMN_WIDTH_INCHES, COLUMN_WIDTH_INCHES))

        # Get reference and actual trajectories
        time = self.stats.time_grid
        n_runs = len(self.reader.flight_data)

        # Select axes based on plane
        if plane == 'xy':
            ref_a, ref_b = self.stats.x_ref, self.stats.y_ref
            err_a_all, err_b_all = self.stats.ex_all, self.stats.ey_all
            label_a, label_b = '$x$ (m)', '$y$ (m)'
        elif plane == 'xz':
            ref_a, ref_b = self.stats.x_ref, self.stats.z_ref
            err_a_all, err_b_all = self.stats.ex_all, self.stats.ez_all
            label_a, label_b = '$x$ (m)', '$z$ (m)'
        else:  # yz
            ref_a, ref_b = self.stats.y_ref, self.stats.z_ref
            err_a_all, err_b_all = self.stats.ey_all, self.stats.ez_all
            label_a, label_b = '$y$ (m)', '$z$ (m)'

        # Calculate actual positions for each run
        actual_a_all = ref_a + err_a_all
        actual_b_all = ref_b + err_b_all

        # Find envelope (min/max across runs at each point)
        min_a = np.min(actual_a_all, axis=0)
        max_a = np.max(actual_a_all, axis=0)
        min_b = np.min(actual_b_all, axis=0)
        max_b = np.max(actual_b_all, axis=0)

        # Create envelope polygon
        # Forward along trajectory with max error, back with min error
        envelope_a = np.concatenate([max_a, min_a[::-1]])
        envelope_b_upper = np.concatenate([max_b, max_b[::-1]])
        envelope_b_lower = np.concatenate([min_b, min_b[::-1]])

        # Plot individual trajectories (light)
        for i in range(n_runs):
            color = RUN_COLORS[i % len(RUN_COLORS)]
            ax.plot(actual_a_all[i], actual_b_all[i],
                   color=color, alpha=0.3, linewidth=0.6,
                   label=self.reader.flight_data[i].name if i < 3 else None)

        # Plot error envelope as shaded region
        # Use convex hull approach for better visualization
        for t_idx in range(0, len(time) - 1, max(1, len(time) // 50)):
            points_a = actual_a_all[:, t_idx]
            points_b = actual_b_all[:, t_idx]
            if len(points_a) > 2:
                ax.scatter(points_a, points_b, color=COLORS['envelope_fill'],
                          alpha=0.1, s=3, marker='.')

        # Plot reference trajectory
        ax.plot(ref_a, ref_b, color=COLORS['nominal'], linewidth=1.5,
               linestyle='--', label='Reference', zorder=10)

        # Start/end markers
        ax.scatter([ref_a[0]], [ref_b[0]], marker='o', s=40,
                  color='green', label='Start', zorder=15)
        ax.scatter([ref_a[-1]], [ref_b[-1]], marker='s', s=40,
                  color='red', label='End', zorder=15)

        ax.set_xlabel(label_a)
        ax.set_ylabel(label_b)
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='best', fontsize=7, ncol=2)
        self._add_minor_ticks(ax)
        ax.grid(True, which='major', alpha=0.5)
        ax.grid(True, which='minor', alpha=0.2)

        fig.tight_layout()

        return self._save_figure(fig, f'error_envelope_{plane}')

    def plot_error_envelope_3d(self, tube_samples: int = 8) -> str:
        """
        Plot 3D trajectory with error envelope (tube visualization).

        Args:
            tube_samples: Number of samples around the circumference of the tube

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig = plt.figure(figsize=(FULL_WIDTH_INCHES, 5.0))
        ax = fig.add_subplot(111, projection='3d')

        time = self.stats.time_grid
        n_runs = len(self.reader.flight_data)

        # Reference trajectory
        x_ref, y_ref, z_ref = self.stats.x_ref, self.stats.y_ref, self.stats.z_ref

        # Plot individual runs
        for i in range(n_runs):
            x_actual = x_ref + self.stats.ex_all[i]
            y_actual = y_ref + self.stats.ey_all[i]
            z_actual = z_ref + self.stats.ez_all[i]
            color = RUN_COLORS[i % len(RUN_COLORS)]
            ax.plot(x_actual, y_actual, z_actual,
                   color=color, alpha=0.4, linewidth=0.6,
                   label=self.reader.flight_data[i].name if i < 4 else None)

        # Plot reference trajectory
        ax.plot(x_ref, y_ref, z_ref, color=COLORS['nominal'],
               linewidth=2.0, linestyle='--', label='Reference')

        # Calculate error radius at each point (max across all runs)
        error_radius = np.max(self.stats.e_norm_all, axis=0)

        # Create error tube visualization (sample points)
        # Subsample for performance
        step = max(1, len(time) // 30)
        theta = np.linspace(0, 2*np.pi, tube_samples)

        for idx in range(0, len(time), step):
            r = error_radius[idx]
            x_c, y_c, z_c = x_ref[idx], y_ref[idx], z_ref[idx]

            # Create circle in xy plane, centered at trajectory point
            circle_x = x_c + r * np.cos(theta)
            circle_y = y_c + r * np.sin(theta)
            circle_z = np.full_like(theta, z_c)

            ax.plot(circle_x, circle_y, circle_z,
                   color=COLORS['envelope_fill'], alpha=0.2, linewidth=0.5)

        # Start/end markers
        ax.scatter([x_ref[0]], [y_ref[0]], [z_ref[0]],
                  marker='o', s=50, color='green', label='Start', zorder=10)
        ax.scatter([x_ref[-1]], [y_ref[-1]], [z_ref[-1]],
                  marker='s', s=50, color='red', label='End', zorder=10)

        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        ax.set_zlabel('$z$ (m)')
        ax.legend(loc='upper left', fontsize=7, ncol=2)

        # Equal aspect ratio
        max_range = max(
            np.ptp(x_ref), np.ptp(y_ref), np.ptp(z_ref)
        ) / 2 * 1.2

        mid_x = (np.max(x_ref) + np.min(x_ref)) / 2
        mid_y = (np.max(y_ref) + np.min(y_ref)) / 2
        mid_z = (np.max(z_ref) + np.min(z_ref)) / 2
        ax.set_xlim(mid_x - max_range, mid_x + max_range)
        ax.set_ylim(mid_y - max_range, mid_y + max_range)
        ax.set_zlim(mid_z - max_range, mid_z + max_range)

        fig.tight_layout()

        return self._save_figure(fig, 'error_envelope_3d')

    def plot_error_envelope_time(self) -> str:
        """
        Plot time series with min/max error envelope.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(4, 1, figsize=(FULL_WIDTH_INCHES, 6.0), sharex=True)

        time = self.stats.time_grid
        errors = [
            ('$e_x$', self.stats.ex_all, COLORS['x']),
            ('$e_y$', self.stats.ey_all, COLORS['y']),
            ('$e_z$', self.stats.ez_all, COLORS['z']),
            (r'$\|e\|$', self.stats.e_norm_all, COLORS['norm']),
        ]

        # Compute shared y-axis limits for per-axis subplots (ex, ey, ez)
        axis_data = [self.stats.ex_all, self.stats.ey_all, self.stats.ez_all]
        y_min = min(np.min(d) for d in axis_data)
        y_max = max(np.max(d) for d in axis_data)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for idx, (ax, (label, data, color)) in enumerate(zip(axes, errors)):
            mean = np.mean(data, axis=0)
            min_val = np.min(data, axis=0)
            max_val = np.max(data, axis=0)

            # Plot envelope (min-max range)
            ax.fill_between(time, min_val, max_val,
                           color=color, alpha=0.2, label='Min-Max')

            # Plot mean
            ax.plot(time, mean, color=color, linewidth=1.2, label='Mean')

            # Plot individual runs (very light)
            for i in range(data.shape[0]):
                ax.plot(time, data[i], color=color, alpha=0.1, linewidth=0.3)

            ax.set_ylabel(f'{label} (m)')
            # Share y-axis limits for per-axis subplots (not the norm)
            if idx < 3:
                ax.set_ylim(shared_ylim)
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

            if ax == axes[0]:
                ax.legend(loc='upper right', fontsize=7, framealpha=0.9)

        axes[-1].set_xlabel('Time (s)')

        fig.tight_layout()

        return self._save_figure(fig, 'error_envelope_time')

    # ==========================================================================
    # Rotation Error Plots (SO(3) e_R)
    # ==========================================================================

    def plot_violin_rotation_error(self) -> str:
        """
        Create violin plots showing SO(3) rotation error distribution.

        The rotation error e_R is computed as: e_R = 1/2 * vee(R_d^T R - R^T R_d)

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(2, 2, figsize=(FULL_WIDTH_INCHES, 5.0))

        # Rotation error data converted to degrees
        error_data = [
            ('$e_{R,x}$', np.degrees(self.stats.e_R_x_all.flatten()), COLORS['x']),
            ('$e_{R,y}$', np.degrees(self.stats.e_R_y_all.flatten()), COLORS['y']),
            ('$e_{R,z}$', np.degrees(self.stats.e_R_z_all.flatten()), COLORS['z']),
            (r'$\|e_R\|$', np.degrees(self.stats.e_R_all.flatten()), COLORS['norm']),
        ]

        # Compute shared y-axis limits for per-axis violins (eRx, eRy, eRz)
        axis_flat = [error_data[0][1], error_data[1][1], error_data[2][1]]
        y_min = min(np.min(d) for d in axis_flat)
        y_max = max(np.max(d) for d in axis_flat)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for idx, (ax, (label, data, color)) in enumerate(zip(axes.flatten(), error_data)):
            # Create violin plot
            parts = ax.violinplot([data], positions=[0], showmeans=False,
                                  showmedians=True, showextrema=False)

            # Style the violin
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.3)

            parts['cmedians'].set_color(VIOLIN_COLORS['median'])
            parts['cmedians'].set_linewidth(1.5)

            # Add box plot overlay for quartiles with whiskers (1.5x IQR)
            bp = ax.boxplot([data], positions=[0], widths=0.15,
                           patch_artist=True, showfliers=False, whis=1.5)
            bp['boxes'][0].set_facecolor('white')
            bp['boxes'][0].set_edgecolor(color)
            bp['boxes'][0].set_alpha(0.8)
            bp['medians'][0].set_color(VIOLIN_COLORS['median'])
            bp['whiskers'][0].set_color(color)
            bp['whiskers'][1].set_color(color)
            bp['caps'][0].set_color(color)
            bp['caps'][1].set_color(color)

            # RMSE annotation as box above the violin
            rmse = np.sqrt(np.mean(data**2))
            ax.annotate(f'RMSE: {rmse:.2f}°',
                       xy=(0, 1.02), xycoords=('data', 'axes fraction'),
                       ha='center', va='bottom', fontsize=7,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor=color, alpha=0.9))

            ax.set_ylabel(f'{label} (deg)')
            # Share y-axis limits for per-axis violins (not the norm)
            if idx < 3:
                ax.set_ylim(shared_ylim)
            ax.set_xticks([])
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout()

        return self._save_figure(fig, 'violin_rotation_error')

    def plot_rotation_error_envelope_time(self) -> str:
        """
        Plot rotation error time series with min/max envelope.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(4, 1, figsize=(FULL_WIDTH_INCHES, 6.0), sharex=True)

        time = self.stats.time_grid
        eRx_deg = np.degrees(self.stats.e_R_x_all)
        eRy_deg = np.degrees(self.stats.e_R_y_all)
        eRz_deg = np.degrees(self.stats.e_R_z_all)
        eR_deg = np.degrees(self.stats.e_R_all)
        errors = [
            ('$e_{R,x}$', eRx_deg, COLORS['x']),
            ('$e_{R,y}$', eRy_deg, COLORS['y']),
            ('$e_{R,z}$', eRz_deg, COLORS['z']),
            (r'$\|e_R\|$', eR_deg, COLORS['norm']),
        ]

        # Compute shared y-axis limits for per-axis subplots (eRx, eRy, eRz)
        axis_data = [eRx_deg, eRy_deg, eRz_deg]
        y_min = min(np.min(d) for d in axis_data)
        y_max = max(np.max(d) for d in axis_data)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for idx, (ax, (label, data, color)) in enumerate(zip(axes, errors)):
            mean = np.mean(data, axis=0)
            min_val = np.min(data, axis=0)
            max_val = np.max(data, axis=0)

            # Plot envelope (min-max range)
            ax.fill_between(time, min_val, max_val,
                           color=color, alpha=0.2, label='Min-Max')

            # Plot mean
            ax.plot(time, mean, color=color, linewidth=1.2, label='Mean')

            # Plot individual runs (very light)
            for i in range(data.shape[0]):
                ax.plot(time, data[i], color=color, alpha=0.1, linewidth=0.3)

            ax.set_ylabel(f'{label} (deg)')
            # Share y-axis limits for per-axis subplots (not the norm)
            if idx < 3:
                ax.set_ylim(shared_ylim)
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

            if ax == axes[0]:
                ax.legend(loc='upper right', fontsize=7, framealpha=0.9)

        axes[-1].set_xlabel('Time (s)')

        # Add overall statistics
        rmse = np.degrees(np.sqrt(np.mean(self.stats.e_R_all**2)))
        max_err = np.degrees(np.max(self.stats.e_R_all))
        axes[-1].annotate(f'Overall RMSE: {rmse:.2f}°, Max: {max_err:.2f}°',
                         xy=(0.98, 0.95), xycoords='axes fraction',
                         ha='right', va='top', fontsize=7,
                         bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

        fig.tight_layout()

        return self._save_figure(fig, 'rotation_error_envelope_time')

    # ==========================================================================
    # Euler Angle Error Plots
    # ==========================================================================

    def plot_violin_quaternion_error(self) -> str:
        """
        Create violin plots showing euler angle error distribution (degrees).

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(1, 3, figsize=(FULL_WIDTH_INCHES, 3.5), sharey=True)

        # Euler angle error data (already in degrees)
        error_data = [
            (r'$e_{\phi}$ (Roll)', self.stats.e_roll_all.flatten(), COLORS['x']),
            (r'$e_{\theta}$ (Pitch)', self.stats.e_pitch_all.flatten(), COLORS['y']),
            (r'$e_{\psi}$ (Yaw)', self.stats.e_yaw_all.flatten(), COLORS['z']),
        ]

        # Compute shared y-axis limits across all euler angle violins
        all_euler_flat = np.concatenate([d for _, d, _ in error_data])
        y_min, y_max = np.min(all_euler_flat), np.max(all_euler_flat)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for ax, (label, data, color) in zip(axes.flatten(), error_data):
            # Create violin plot
            parts = ax.violinplot([data], positions=[0], showmeans=False,
                                  showmedians=True, showextrema=False)

            # Style the violin
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.3)

            parts['cmedians'].set_color(VIOLIN_COLORS['median'])
            parts['cmedians'].set_linewidth(1.5)

            # Add box plot overlay for quartiles with whiskers (1.5x IQR)
            bp = ax.boxplot([data], positions=[0], widths=0.15,
                           patch_artist=True, showfliers=False, whis=1.5)
            bp['boxes'][0].set_facecolor('white')
            bp['boxes'][0].set_edgecolor(color)
            bp['boxes'][0].set_alpha(0.8)
            bp['medians'][0].set_color(VIOLIN_COLORS['median'])
            bp['whiskers'][0].set_color(color)
            bp['whiskers'][1].set_color(color)
            bp['caps'][0].set_color(color)
            bp['caps'][1].set_color(color)

            # RMSE annotation as box above the violin
            rmse = np.sqrt(np.mean(data**2))
            ax.annotate(f'RMSE: {rmse:.2f}°',
                       xy=(0, 1.02), xycoords=('data', 'axes fraction'),
                       ha='center', va='bottom', fontsize=7,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor=color, alpha=0.9))

            ax.set_ylabel('Euler Error (deg)')
            ax.set_xlabel(label)
            ax.set_ylim(shared_ylim)
            ax.set_xticks([])
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout()

        # Re-apply shared y-axis limits after tight_layout to prevent autoscaling
        for ax in axes.flatten():
            ax.set_ylim(shared_ylim)

        return self._save_figure(fig, 'violin_euler_error')

    def plot_combined_violin_error(self) -> str:
        """
        Create a single figure with position (x,y,z) and attitude (roll,pitch,yaw)
        violin plots, each with RMSE annotation boxes.

        Top subplot: x, y, z position error violins side-by-side
        Bottom subplot: roll, pitch, yaw euler angle error violins side-by-side

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, (ax_pos, ax_att) = plt.subplots(
            2, 1, figsize=(COLUMN_WIDTH_INCHES, 4.5),
            gridspec_kw={'hspace': 0.45}
        )

        # --- Helper to draw a single violin + box + RMSE at a position ---
        def _draw_violin(ax, data, position, color, label, unit, rmse_fmt):
            parts = ax.violinplot([data], positions=[position], showmeans=False,
                                  showmedians=True, showextrema=False, widths=0.7)
            for pc in parts['bodies']:
                pc.set_facecolor(color)
                pc.set_edgecolor(color)
                pc.set_alpha(0.3)
            parts['cmedians'].set_color(VIOLIN_COLORS['median'])
            parts['cmedians'].set_linewidth(1.5)

            # Add box plot overlay for quartiles with whiskers (1.5x IQR)
            bp = ax.boxplot([data], positions=[position], widths=0.18,
                           patch_artist=True, showfliers=False, whis=1.5)
            bp['boxes'][0].set_facecolor('white')
            bp['boxes'][0].set_edgecolor(color)
            bp['boxes'][0].set_alpha(0.8)
            bp['medians'][0].set_color(VIOLIN_COLORS['median'])
            for w in bp['whiskers']:
                w.set_color(color)
            for c in bp['caps']:
                c.set_color(color)

            rmse = np.sqrt(np.mean(data**2))
            ax.annotate(rmse_fmt.format(rmse=rmse),
                       xy=(position, 1.02), xycoords=('data', 'axes fraction'),
                       ha='center', va='bottom', fontsize=6.5,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor=color, alpha=0.9))

        # ==================================================================
        # Top subplot: Position errors (x, y, z)
        # ==================================================================
        pos_errors = [
            (0, '$e_x$', self.stats.ex_all.flatten(), COLORS['x']),
            (1, '$e_y$', self.stats.ey_all.flatten(), COLORS['y']),
            (2, '$e_z$', self.stats.ez_all.flatten(), COLORS['z']),
        ]

        for pos, label, data, color in pos_errors:
            _draw_violin(ax_pos, data, pos, color, label, 'm',
                        'RMSE: {rmse:.4f} m')

        ax_pos.set_xticks([0, 1, 2])
        ax_pos.set_xticklabels(['$e_x$', '$e_y$', '$e_z$'])
        ax_pos.set_ylabel('Position Error (m)')
        ax_pos.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
        self._add_minor_ticks(ax_pos)
        ax_pos.grid(True, which='major', alpha=0.5, axis='y')

        # ==================================================================
        # Bottom subplot: Euler angle errors (roll, pitch, yaw)
        # ==================================================================
        att_errors = [
            (0, r'$e_{\phi}$', self.stats.e_roll_all.flatten(), COLORS['x']),
            (1, r'$e_{\theta}$', self.stats.e_pitch_all.flatten(), COLORS['y']),
            (2, r'$e_{\psi}$', self.stats.e_yaw_all.flatten(), COLORS['z']),
        ]

        for pos, label, data, color in att_errors:
            _draw_violin(ax_att, data, pos, color, label, 'deg',
                        'RMSE: {rmse:.2f}°')

        ax_att.set_xticks([0, 1, 2])
        ax_att.set_xticklabels([r'$e_{\phi}$ (Roll)', r'$e_{\theta}$ (Pitch)',
                                r'$e_{\psi}$ (Yaw)'])
        ax_att.set_ylabel('Attitude Error (deg)')
        ax_att.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
        self._add_minor_ticks(ax_att)
        ax_att.grid(True, which='major', alpha=0.5, axis='y')

        fig.tight_layout(rect=[0, 0, 1, 0.95])
        fig.subplots_adjust(hspace=0.2)

        return self._save_figure(fig, 'violin_combined_error')

    def plot_quaternion_error_envelope_time(self) -> str:
        """
        Plot euler angle error time series with min/max envelope (degrees).

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, axes = plt.subplots(3, 1, figsize=(FULL_WIDTH_INCHES, 5.0), sharex=True)

        time = self.stats.time_grid
        # Euler angle errors (already in degrees)
        errors = [
            (r'$e_{\phi}$ (Roll)', self.stats.e_roll_all, COLORS['x']),
            (r'$e_{\theta}$ (Pitch)', self.stats.e_pitch_all, COLORS['y']),
            (r'$e_{\psi}$ (Yaw)', self.stats.e_yaw_all, COLORS['z']),
        ]

        # Compute shared y-axis range across all euler angle subplots
        all_euler = [self.stats.e_roll_all, self.stats.e_pitch_all, self.stats.e_yaw_all]
        y_min = min(np.min(d) for d in all_euler)
        y_max = max(np.max(d) for d in all_euler)
        y_margin = (y_max - y_min) * 0.1
        shared_ylim = (y_min - y_margin, y_max + y_margin)

        for ax, (label, data, color) in zip(axes, errors):
            mean = np.mean(data, axis=0)
            min_val = np.min(data, axis=0)
            max_val = np.max(data, axis=0)

            # Plot envelope (min-max range)
            ax.fill_between(time, min_val, max_val,
                           color=color, alpha=0.2, label='Min-Max')

            # Plot mean
            ax.plot(time, mean, color=color, linewidth=1.2, label='Mean')

            # Plot individual runs (very light)
            for i in range(data.shape[0]):
                ax.plot(time, data[i], color=color, alpha=0.1, linewidth=0.3)

            ax.set_ylabel(f'{label} (deg)')
            ax.set_ylim(shared_ylim)
            ax.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

            # RMSE annotation box at top of each subplot
            rmse = np.sqrt(np.mean(data**2))
            ax.annotate(f'RMSE: {rmse:.2f}°',
                       xy=(0.98, 0.95), xycoords='axes fraction',
                       ha='right', va='top', fontsize=7,
                       bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                                 edgecolor=color, alpha=0.9))

            if ax == axes[0]:
                ax.legend(loc='upper left', fontsize=7, framealpha=0.9)

        axes[-1].set_xlabel('Time (s)')

        fig.tight_layout()

        return self._save_figure(fig, 'euler_error_envelope_time')

    def plot_combined_attitude_error(self) -> str:
        """
        Create a combined figure showing both rotation error and euler angle error.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig = plt.figure(figsize=(FULL_WIDTH_INCHES, 5.5))
        gs = fig.add_gridspec(2, 2, hspace=0.25, wspace=0.25)

        time = self.stats.time_grid

        # Top left: ||e_R|| time series with envelope (degrees)
        ax1 = fig.add_subplot(gs[0, 0])
        data = np.degrees(self.stats.e_R_all)
        mean = np.mean(data, axis=0)
        min_val = np.min(data, axis=0)
        max_val = np.max(data, axis=0)
        ax1.fill_between(time, min_val, max_val, color=COLORS['norm'], alpha=0.2)
        ax1.plot(time, mean, color=COLORS['norm'], linewidth=1.2)
        ax1.set_ylabel(r'$\|e_R\|$ (deg)')
        ax1.set_xlabel('Time (s)')
        self._add_minor_ticks(ax1)
        ax1.grid(True, which='major', alpha=0.5)

        # Top right: Violin plot for ||e_R|| (degrees)
        ax2 = fig.add_subplot(gs[0, 1])
        parts = ax2.violinplot([data.flatten()], positions=[0], showmeans=False,
                               showmedians=True, showextrema=False)
        for pc in parts['bodies']:
            pc.set_facecolor(COLORS['norm'])
            pc.set_alpha(0.3)
        parts['cmedians'].set_color(VIOLIN_COLORS['median'])
        rmse = np.sqrt(np.mean(data**2))
        ax2.annotate(f'RMSE: {rmse:.2f}°',
                    xy=(0, 1.02), xycoords=('data', 'axes fraction'),
                    ha='center', va='bottom', fontsize=7,
                    bbox=dict(boxstyle='round,pad=0.3', facecolor='white',
                              edgecolor=COLORS['norm'], alpha=0.9))
        ax2.set_ylabel(r'$\|e_R\|$ (deg)')
        ax2.set_xticks([])
        ax2.grid(True, which='major', alpha=0.5, axis='y')

        # Bottom left: Euler angle error time series
        ax3 = fig.add_subplot(gs[1, 0])
        euler_data = [
            (self.stats.e_roll_all, COLORS['x'], r'$e_{\phi}$ (Roll)'),
            (self.stats.e_pitch_all, COLORS['y'], r'$e_{\theta}$ (Pitch)'),
            (self.stats.e_yaw_all, COLORS['z'], r'$e_{\psi}$ (Yaw)'),
        ]
        for data, color, label in euler_data:
            mean = np.mean(data, axis=0)
            ax3.plot(time, mean, color=color, linewidth=1.0, label=label)
            ax3.fill_between(time, np.min(data, axis=0), np.max(data, axis=0),
                            color=color, alpha=0.15)
        ax3.set_ylabel('Euler Angle Error (deg)')
        ax3.set_xlabel('Time (s)')
        ax3.legend(loc='upper right', fontsize=6, ncol=3)
        ax3.axhline(y=0, color='black', linewidth=0.3, alpha=0.3)
        self._add_minor_ticks(ax3)
        ax3.grid(True, which='major', alpha=0.5)

        # Bottom right: Violin plots for euler angle components
        ax4 = fig.add_subplot(gs[1, 1])
        euler_errors = [
            self.stats.e_roll_all.flatten(),
            self.stats.e_pitch_all.flatten(),
            self.stats.e_yaw_all.flatten(),
        ]
        colors_e = [COLORS['x'], COLORS['y'], COLORS['z']]
        positions = [0, 1, 2]
        parts = ax4.violinplot(euler_errors, positions=positions, showmeans=False,
                               showmedians=True, showextrema=False)
        for i, pc in enumerate(parts['bodies']):
            pc.set_facecolor(colors_e[i])
            pc.set_alpha(0.3)
        parts['cmedians'].set_color(VIOLIN_COLORS['median'])
        ax4.set_xticks(positions)
        ax4.set_xticklabels([r'$e_{\phi}$ (Roll)', r'$e_{\theta}$ (Pitch)', r'$e_{\psi}$ (Yaw)'])
        ax4.set_ylabel('Euler Error (deg)')

        # Shared y-axis limits across euler angle violins
        all_euler = np.concatenate(euler_errors)
        e_ymin, e_ymax = np.min(all_euler), np.max(all_euler)
        e_margin = (e_ymax - e_ymin) * 0.1
        ax4.set_ylim(e_ymin - e_margin, e_ymax + e_margin)

        ax4.axhline(y=0, color='black', linewidth=0.3, alpha=0.3)
        ax4.grid(True, which='major', alpha=0.5, axis='y')

        # Add RMSE annotations above each violin
        for i, (data, color) in enumerate(zip(euler_errors, colors_e)):
            rmse = np.sqrt(np.mean(data**2))
            ax4.annotate(f'RMSE: {rmse:.2f}°',
                        xy=(i, 1.02), xycoords=('data', 'axes fraction'),
                        ha='center', va='bottom', fontsize=6,
                        bbox=dict(boxstyle='round,pad=0.2', facecolor='white',
                                  edgecolor=color, alpha=0.9))

        fig.tight_layout()

        return self._save_figure(fig, 'combined_attitude_error')

    # ==========================================================================
    # Combined / Summary Plots
    # ==========================================================================

    def plot_summary_statistics_table(self) -> str:
        """
        Create a figure with summary statistics table.

        Returns:
            Path to saved figure
        """
        if self.stats is None:
            self.compute_statistics()

        fig, ax = plt.subplots(figsize=(FULL_WIDTH_INCHES, 3.0))
        ax.axis('off')

        # Calculate statistics
        n_runs = len(self.reader.flight_data)
        metrics = ['RMSE (m)', 'Mean (m)', 'Std (m)', 'Max (m)', 'Min (m)']
        axes_labels = ['$e_x$', '$e_y$', '$e_z$', r'$\|e\|$']
        data_arrays = [
            self.stats.ex_all.flatten(),
            self.stats.ey_all.flatten(),
            self.stats.ez_all.flatten(),
            self.stats.e_norm_all.flatten(),
        ]

        # Build table data
        table_data = []
        for metric in metrics:
            row = []
            for data in data_arrays:
                if metric == 'RMSE (cm)':
                    val = np.sqrt(np.mean(data**2))
                elif metric == 'Mean (cm)':
                    val = np.mean(data)
                elif metric == 'Std (cm)':
                    val = np.std(data)
                elif metric == 'Max (cm)':
                    val = np.max(data)
                else:  # Min
                    val = np.min(data)
                row.append(f'{val:.3f}')
            table_data.append(row)

        # Create table
        table = ax.table(cellText=table_data,
                        rowLabels=metrics,
                        colLabels=axes_labels,
                        cellLoc='center',
                        rowLoc='center',
                        loc='center')

        table.auto_set_font_size(False)
        table.set_fontsize(9)
        table.scale(1.2, 1.5)

        # Style header
        for j, label in enumerate(axes_labels):
            table[(0, j)].set_facecolor(COLORS['sky_blue'])
            table[(0, j)].set_text_props(weight='bold')

        # Style row labels
        for i, _ in enumerate(metrics):
            table[(i+1, -1)].set_facecolor('#f0f0f0')


        fig.tight_layout()

        return self._save_figure(fig, 'summary_statistics')

    def plot_all(self, plot_types: Optional[List[str]] = None) -> List[str]:
        """
        Generate all or selected plots.

        Args:
            plot_types: List of plot types to generate. Options:
                'violin', 'error_bars', 'envelope', 'rotation', 'quaternion', 'summary'
                If None, generates all.

        Returns:
            List of paths to saved figures
        """
        if plot_types is None:
            plot_types = ['violin', 'error_bars', 'envelope', 'rotation', 'quaternion', 'summary']

        paths = []

        print("\nComputing error statistics...")
        self.compute_statistics()

        print("\nGenerating plots...")

        if 'violin' in plot_types:
            paths.append(self.plot_violin_position_error())
            paths.append(self.plot_violin_per_run())

        if 'error_bars' in plot_types:
            paths.append(self.plot_error_bars_time_series(error_type='std'))
            paths.append(self.plot_error_bars_time_series(error_type='ci'))
            paths.append(self.plot_error_bars_summary())

        if 'envelope' in plot_types:
            paths.append(self.plot_error_envelope_2d('xy'))
            paths.append(self.plot_error_envelope_2d('xz'))
            paths.append(self.plot_error_envelope_3d())
            paths.append(self.plot_error_envelope_time())

        if 'rotation' in plot_types:
            paths.append(self.plot_violin_rotation_error())
            paths.append(self.plot_rotation_error_envelope_time())

        if 'quaternion' in plot_types:
            paths.append(self.plot_violin_quaternion_error())
            paths.append(self.plot_quaternion_error_envelope_time())
            paths.append(self.plot_combined_attitude_error())
            paths.append(self.plot_combined_violin_error())

        if 'summary' in plot_types:
            paths.append(self.plot_summary_statistics_table())

        print(f"\nGenerated {len(paths)} figures in {self.output_dir}/")
        return paths


# ==============================================================================
# Flight Window Detection
# ==============================================================================

@dataclass
class FlightWindow:
    """Detected flight window for a single bag."""
    bag_path: str
    bag_name: str
    start_time: float  # Relative to bag start
    end_time: float    # Relative to bag start
    duration: float
    start_altitude: float
    end_altitude: float
    max_altitude: float


def detect_flight_window(bag_path: str, start_altitude: float, end_altitude: float,
                         use_setpoint: bool = False) -> Optional[FlightWindow]:
    """
    Detect flight window based on altitude thresholds.

    Args:
        bag_path: Path to the ROS bag file
        start_altitude: Altitude threshold for flight start (m)
        end_altitude: Altitude threshold for flight end (m)
        use_setpoint: If True, use setpoint altitude instead of actual

    Returns:
        FlightWindow with detected times, or None if detection failed
    """
    timestamps = []
    altitudes = []

    with rosbag.Bag(bag_path, 'r') as bag:
        bag_start = bag.get_start_time()

        topic = '/mavros/setpoint_raw/local' if use_setpoint else '/mavros/odometry/in'

        for _, msg, t in bag.read_messages(topics=[topic]):
            rel_time = t.to_sec() - bag_start
            timestamps.append(rel_time)

            if use_setpoint:
                altitudes.append(msg.position.z)
            else:
                altitudes.append(msg.pose.pose.position.z)

    if len(timestamps) < 10:
        print(f"  Warning: Not enough data points in {bag_path}")
        return None

    timestamps = np.array(timestamps)
    altitudes = np.array(altitudes)

    # Find start time: first time altitude crosses start_altitude (going up)
    start_idx = None
    for i in range(1, len(altitudes)):
        if altitudes[i-1] < start_altitude <= altitudes[i]:
            start_idx = i
            break

    if start_idx is None:
        # Try finding where altitude first exceeds threshold
        above_start = np.where(altitudes >= start_altitude)[0]
        if len(above_start) > 0:
            start_idx = above_start[0]
        else:
            print(f"  Warning: Never reached start altitude {start_altitude}m in {bag_path}")
            return None

    # Find end time: last time altitude crosses end_altitude (going down) after start
    end_idx = None
    for i in range(len(altitudes) - 1, start_idx, -1):
        if altitudes[i] < end_altitude <= altitudes[i-1]:
            end_idx = i
            break

    if end_idx is None:
        # Try finding where altitude last exceeds threshold
        above_end = np.where(altitudes[start_idx:] >= end_altitude)[0]
        if len(above_end) > 0:
            end_idx = start_idx + above_end[-1]
        else:
            print(f"  Warning: Could not find end altitude {end_altitude}m in {bag_path}")
            end_idx = len(altitudes) - 1

    start_time = timestamps[start_idx]
    end_time = timestamps[end_idx]

    return FlightWindow(
        bag_path=bag_path,
        bag_name=os.path.basename(bag_path),
        start_time=start_time,
        end_time=end_time,
        duration=end_time - start_time,
        start_altitude=altitudes[start_idx],
        end_altitude=altitudes[end_idx],
        max_altitude=np.max(altitudes[start_idx:end_idx+1]) if end_idx > start_idx else altitudes[start_idx]
    )


def inspect_bags(bag_paths: List[str], start_altitude: float, end_altitude: float,
                 use_setpoint: bool = False, output_dir: str = './figures',
                 format: str = 'pdf', dpi: int = 300) -> List[FlightWindow]:
    """
    Inspect multiple bags, detect flight windows, and generate visualization.

    Args:
        bag_paths: List of bag file paths
        start_altitude: Altitude threshold for flight start (m)
        end_altitude: Altitude threshold for flight end (m)
        use_setpoint: If True, use setpoint altitude
        output_dir: Directory to save inspection figure
        format: Output format
        dpi: Output DPI

    Returns:
        List of detected FlightWindow objects
    """
    print(f"\nInspecting {len(bag_paths)} bag(s)...")
    print(f"  Start altitude threshold: {start_altitude} m")
    print(f"  End altitude threshold: {end_altitude} m")
    print(f"  Using: {'setpoint' if use_setpoint else 'actual'} altitude\n")

    windows = []
    all_data = []  # For plotting: (timestamps, altitudes, bag_name)

    for bag_path in bag_paths:
        print(f"Processing: {os.path.basename(bag_path)}")

        # Detect flight window
        window = detect_flight_window(bag_path, start_altitude, end_altitude, use_setpoint)
        if window:
            windows.append(window)
            print(f"  Flight window: {window.start_time:.2f}s - {window.end_time:.2f}s "
                  f"(duration: {window.duration:.2f}s)")
            print(f"  Altitude range: {window.start_altitude:.2f}m - {window.max_altitude:.2f}m - {window.end_altitude:.2f}m")

        # Collect data for plotting
        timestamps = []
        altitudes = []
        altitudes_sp = []

        with rosbag.Bag(bag_path, 'r') as bag:
            bag_start = bag.get_start_time()

            for _, msg, t in bag.read_messages(topics=['/mavros/odometry/in']):
                timestamps.append(t.to_sec() - bag_start)
                altitudes.append(msg.pose.pose.position.z)

            for _, msg, t in bag.read_messages(topics=['/mavros/setpoint_raw/local']):
                t_rel = t.to_sec() - bag_start
                # Interpolate to match odometry timestamps
                altitudes_sp.append((t_rel, msg.position.z))

        all_data.append({
            'name': os.path.basename(bag_path),
            'timestamps': np.array(timestamps),
            'altitudes': np.array(altitudes),
            'altitudes_sp': altitudes_sp,
            'window': window
        })

    # Generate inspection plot
    setup_publication_style()
    os.makedirs(output_dir, exist_ok=True)

    n_bags = len(all_data)
    fig, axes = plt.subplots(n_bags, 1, figsize=(FULL_WIDTH_INCHES, 2.5 * n_bags),
                              sharex=False, squeeze=False)

    for i, data in enumerate(all_data):
        ax = axes[i, 0]
        t = data['timestamps']
        z = data['altitudes']
        window = data['window']

        # Plot actual altitude
        ax.plot(t, z, color=COLORS['blue'], linewidth=1.0, label='Actual altitude')

        # Plot setpoint if available
        if data['altitudes_sp']:
            t_sp = np.array([x[0] for x in data['altitudes_sp']])
            z_sp = np.array([x[1] for x in data['altitudes_sp']])
            ax.plot(t_sp, z_sp, color=COLORS['orange'], linewidth=0.8,
                   linestyle='--', label='Setpoint', alpha=0.7)

        # Draw threshold lines
        ax.axhline(y=start_altitude, color=COLORS['green'], linestyle=':',
                  linewidth=1.5, label=f'Start threshold ({start_altitude}m)')
        ax.axhline(y=end_altitude, color=COLORS['vermillion'], linestyle=':',
                  linewidth=1.5, label=f'End threshold ({end_altitude}m)')

        # Highlight detected flight window
        if window:
            ax.axvline(x=window.start_time, color=COLORS['green'], linewidth=1.5, alpha=0.8)
            ax.axvline(x=window.end_time, color=COLORS['vermillion'], linewidth=1.5, alpha=0.8)
            ax.axvspan(window.start_time, window.end_time, alpha=0.15, color=COLORS['green'],
                      label=f'Flight window ({window.duration:.1f}s)')

        ax.set_ylabel('Altitude (m)')
        ax.set_xlabel('Time (s)')
        ax.set_title(f'{data["name"]}', fontsize=9)
        ax.legend(loc='upper right', fontsize=7, ncol=2)
        ax.grid(True, alpha=0.5)
        ax.set_ylim(bottom=min(0, np.min(z) - 0.1))

    fig.tight_layout()

    plot_path = os.path.join(output_dir, f'flight_inspection.{format}')
    fig.savefig(plot_path, dpi=dpi, format=format)
    plt.close(fig)
    print(f"\nSaved inspection plot: {plot_path}")

    # Print summary table
    print("\n" + "=" * 80)
    print("FLIGHT WINDOW SUMMARY")
    print("=" * 80)
    print(f"{'Bag File':<40} {'Start (s)':<12} {'End (s)':<12} {'Duration (s)':<12}")
    print("-" * 80)

    for w in windows:
        print(f"{w.bag_name:<40} {w.start_time:<12.2f} {w.end_time:<12.2f} {w.duration:<12.2f}")

    if windows:
        # Find common window (max of starts, min of ends)
        common_start = max(w.start_time for w in windows)
        common_end = min(w.end_time for w in windows)

        print("-" * 80)
        if common_end > common_start:
            print(f"{'COMMON WINDOW:':<40} {common_start:<12.2f} {common_end:<12.2f} {common_end - common_start:<12.2f}")
            print("\nTo analyze with these bounds, run:")
            print(f"  python {sys.argv[0]} {' '.join(bag_paths)} --start {common_start:.2f} --end {common_end:.2f}")
        else:
            print("WARNING: No common flight window found across all bags!")

    print("=" * 80)

    return windows


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Analyze error distributions across multiple ROS bag flight recordings.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s run1.bag run2.bag run3.bag
  %(prog)s *.bag --output figures/ --plot-type violin
  %(prog)s *.bag --plot-type envelope --format png --dpi 600
  %(prog)s *.bag --names "Trial 1" "Trial 2" "Trial 3"
        """
    )

    parser.add_argument('bag_files', nargs='+',
                        help='ROS bag files to analyze')
    parser.add_argument('--names', '-n', nargs='+', default=None,
                        help='Names for each run (for legends)')
    parser.add_argument('--output', '-o', default='./figures',
                        help='Output directory for figures (default: ./figures)')
    parser.add_argument('--start', '-s', type=float, default=None,
                        help='Start time in seconds (relative to bag start)')
    parser.add_argument('--end', '-e', type=float, default=None,
                        help='End time in seconds (relative to bag start)')
    parser.add_argument('--dpi', type=int, default=300,
                        help='DPI for raster output (default: 300)')
    parser.add_argument('--format', '-f', choices=['pdf', 'png', 'eps', 'svg'],
                        default='pdf', help='Output format (default: pdf)')
    parser.add_argument('--plot-type', '-p', nargs='+',
                        choices=['violin', 'error_bars', 'envelope', 'rotation', 'quaternion', 'summary', 'all'],
                        default=['all'],
                        help='Types of plots to generate (default: all). Options: violin (position error violins), error_bars (time series with std/CI), envelope (2D/3D trajectory envelopes), rotation (SO(3) e_R plots), quaternion (quaternion component errors), summary (statistics table)')
    parser.add_argument('--time-resolution', type=float, default=0.01,
                        help='Time resolution for resampling in seconds (default: 0.01)')

    # Inspection and auto-detection options
    parser.add_argument('--inspect', action='store_true',
                        help='Inspect bags and detect flight windows based on altitude thresholds')
    parser.add_argument('--start-altitude', type=float, default=None,
                        help='Starting altitude threshold (m) - flight starts when drone reaches this altitude')
    parser.add_argument('--end-altitude', type=float, default=None,
                        help='Ending altitude threshold (m) - flight ends when drone descends to this altitude')
    parser.add_argument('--use-setpoint', action='store_true',
                        help='Use setpoint altitude instead of actual altitude for detection')
    parser.add_argument('--run-analysis', action='store_true',
                        help='After detecting flight windows, automatically run the error analysis using per-bag times')

    args = parser.parse_args()

    # Verify bag files exist
    for bag_file in args.bag_files:
        if not os.path.exists(bag_file):
            print(f"Error: Bag file not found: {bag_file}")
            sys.exit(1)

    # Handle inspect mode
    if args.inspect or args.run_analysis:
        if args.start_altitude is None or args.end_altitude is None:
            print("Error: --inspect/--run-analysis requires both --start-altitude and --end-altitude")
            sys.exit(1)

        windows = inspect_bags(
            bag_paths=args.bag_files,
            start_altitude=args.start_altitude,
            end_altitude=args.end_altitude,
            use_setpoint=args.use_setpoint,
            output_dir=args.output,
            format=args.format,
            dpi=args.dpi
        )

        if not args.run_analysis:
            return  # Exit after inspection only

        # Continue with analysis using per-bag times
        if len(windows) != len(args.bag_files):
            print(f"\nError: Could not detect flight windows for all bags "
                  f"({len(windows)}/{len(args.bag_files)})")
            sys.exit(1)

        print("\n" + "=" * 80)
        print("RUNNING ERROR ANALYSIS WITH PER-BAG FLIGHT WINDOWS")
        print("=" * 80)

        # Build per-bag times from detected windows
        per_bag_times = [(w.start_time, w.end_time) for w in windows]

        # Read all bags with per-bag times
        reader = MultiBagReader(args.bag_files, args.names)
        reader.read_all(per_bag_times=per_bag_times)

        # Create plotter and generate plots
        plotter = ErrorDistributionPlotter(reader, output_dir=args.output,
                                            dpi=args.dpi, format=args.format)
        plot_types = None if 'all' in args.plot_type else args.plot_type
        plotter.plot_all(plot_types)
        return

    if len(args.bag_files) < 2:
        print("Warning: Only one bag file provided. Error distribution analysis works best with multiple runs.")

    # Validate names if provided
    if args.names and len(args.names) != len(args.bag_files):
        print(f"Error: Number of names ({len(args.names)}) must match number of bag files ({len(args.bag_files)})")
        sys.exit(1)

    # Read all bags
    reader = MultiBagReader(args.bag_files, args.names)
    reader.read_all(start_time=args.start, end_time=args.end)

    # Create plotter
    plotter = ErrorDistributionPlotter(reader, output_dir=args.output,
                                        dpi=args.dpi, format=args.format)

    # Generate plots
    plot_types = None if 'all' in args.plot_type else args.plot_type
    plotter.plot_all(plot_types)


if __name__ == '__main__':
    main()
