#!/usr/bin/env python3
"""
Professional ROS Bag Telemetry Visualizer for Academic Papers.

This script reads compressed ROS1 bags and generates publication-quality plots
for drone telemetry data including position, velocity, attitude, and setpoints.

Usage:
    python rosbag_plotter.py <bag_file> [options]
    python rosbag_plotter.py flight_data.bag --output figures/
    python rosbag_plotter.py flight_data.bag --start 10 --end 60 --dpi 600

Author: IRAL Lab
"""

import argparse
import os
import sys
from dataclasses import dataclass, field
from typing import Dict, List, Optional, Tuple
import numpy as np

# Plotting imports
import matplotlib
matplotlib.use('Agg')  # Non-interactive backend for saving figures
import matplotlib.pyplot as plt
from matplotlib.ticker import AutoMinorLocator, MaxNLocator
from mpl_toolkits.mplot3d import Axes3D

# ROS bag reading
import rosbag


# ==============================================================================
# Publication-Quality Plot Configuration
# ==============================================================================

# IEEE/Conference standard: column width ~3.5in, full width ~7in
COLUMN_WIDTH_INCHES = 3.5
FULL_WIDTH_INCHES = 7.16
GOLDEN_RATIO = 1.618

# Color palette - colorblind-friendly (Wong, 2011)
COLORS = {
    'actual': '#0072B2',      # Blue
    'setpoint': '#D55E00',    # Vermillion/Orange
    'error': '#009E73',       # Bluish green
    'x': '#0072B2',           # Blue
    'y': '#D55E00',           # Vermillion
    'z': '#009E73',           # Bluish green
    'roll': '#E69F00',        # Orange
    'pitch': '#56B4E9',       # Sky blue
    'yaw': '#CC79A7',         # Reddish purple
    'thrust': '#F0E442',      # Yellow
    'grid': '#CCCCCC',
}

# Line styles
LINE_STYLES = {
    'actual': '-',
    'setpoint': '--',
}

LINE_WIDTHS = {
    'actual': 1.2,
    'setpoint': 1.0,
}


def setup_publication_style():
    """Configure matplotlib for publication-quality figures."""
    plt.rcParams.update({
        # Font settings - use Computer Modern (LaTeX default) or Times
        'font.family': 'serif',
        'font.serif': ['Times New Roman', 'Times', 'DejaVu Serif', 'Computer Modern Roman'],
        'font.size': 9,
        'axes.titlesize': 10,
        'axes.labelsize': 9,
        'xtick.labelsize': 8,
        'ytick.labelsize': 8,
        'legend.fontsize': 8,

        # LaTeX rendering (set to False if LaTeX not available)
        'text.usetex': False,
        'mathtext.fontset': 'cm',  # Computer Modern for math

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
class OdometryData:
    """Container for odometry data from /mavros/odometry/in."""
    timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    # Position (ENU frame)
    x: np.ndarray = field(default_factory=lambda: np.array([]))
    y: np.ndarray = field(default_factory=lambda: np.array([]))
    z: np.ndarray = field(default_factory=lambda: np.array([]))
    # Velocity (ENU frame)
    vx: np.ndarray = field(default_factory=lambda: np.array([]))
    vy: np.ndarray = field(default_factory=lambda: np.array([]))
    vz: np.ndarray = field(default_factory=lambda: np.array([]))
    # Orientation (quaternion)
    qx: np.ndarray = field(default_factory=lambda: np.array([]))
    qy: np.ndarray = field(default_factory=lambda: np.array([]))
    qz: np.ndarray = field(default_factory=lambda: np.array([]))
    qw: np.ndarray = field(default_factory=lambda: np.array([]))
    # Euler angles (computed)
    roll: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw: np.ndarray = field(default_factory=lambda: np.array([]))


@dataclass
class PositionSetpointData:
    """Container for position setpoint data from /mavros/setpoint_raw/local."""
    timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    # Position setpoint
    x: np.ndarray = field(default_factory=lambda: np.array([]))
    y: np.ndarray = field(default_factory=lambda: np.array([]))
    z: np.ndarray = field(default_factory=lambda: np.array([]))
    # Velocity setpoint
    vx: np.ndarray = field(default_factory=lambda: np.array([]))
    vy: np.ndarray = field(default_factory=lambda: np.array([]))
    vz: np.ndarray = field(default_factory=lambda: np.array([]))
    # Acceleration setpoint
    ax: np.ndarray = field(default_factory=lambda: np.array([]))
    ay: np.ndarray = field(default_factory=lambda: np.array([]))
    az: np.ndarray = field(default_factory=lambda: np.array([]))
    # Yaw setpoint
    yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw_rate: np.ndarray = field(default_factory=lambda: np.array([]))


@dataclass
class AttitudeSetpointData:
    """Container for attitude setpoint data from /mavros/setpoint_raw/attitude."""
    timestamps: np.ndarray = field(default_factory=lambda: np.array([]))
    # Orientation setpoint (quaternion)
    qx: np.ndarray = field(default_factory=lambda: np.array([]))
    qy: np.ndarray = field(default_factory=lambda: np.array([]))
    qz: np.ndarray = field(default_factory=lambda: np.array([]))
    qw: np.ndarray = field(default_factory=lambda: np.array([]))
    # Euler angles (computed)
    roll: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw: np.ndarray = field(default_factory=lambda: np.array([]))
    # Body rates setpoint
    roll_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    pitch_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    yaw_rate: np.ndarray = field(default_factory=lambda: np.array([]))
    # Thrust
    thrust: np.ndarray = field(default_factory=lambda: np.array([]))


# ==============================================================================
# Quaternion Utilities
# ==============================================================================

def quaternion_to_euler(qx: float, qy: float, qz: float, qw: float) -> Tuple[float, float, float]:
    """
    Convert quaternion to Euler angles (roll, pitch, yaw).

    Uses aerospace convention (ZYX rotation order).

    Args:
        qx, qy, qz, qw: Quaternion components

    Returns:
        Tuple of (roll, pitch, yaw) in radians
    """
    # Roll (x-axis rotation)
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch (y-axis rotation)
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)  # Clamp for numerical stability
    pitch = np.arcsin(sinp)

    # Yaw (z-axis rotation)
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def quaternion_to_euler_vectorized(qx: np.ndarray, qy: np.ndarray,
                                    qz: np.ndarray, qw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Vectorized quaternion to Euler conversion."""
    # Roll
    sinr_cosp = 2.0 * (qw * qx + qy * qz)
    cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    # Pitch
    sinp = 2.0 * (qw * qy - qz * qx)
    sinp = np.clip(sinp, -1.0, 1.0)
    pitch = np.arcsin(sinp)

    # Yaw
    siny_cosp = 2.0 * (qw * qz + qx * qy)
    cosy_cosp = 1.0 - 2.0 * (qy * qy + qz * qz)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw


def unwrap_angle(angles: np.ndarray) -> np.ndarray:
    """Unwrap angles to avoid discontinuities at ±π."""
    return np.unwrap(angles)


def normalize_quaternion_continuous(qx: np.ndarray, qy: np.ndarray,
                                     qz: np.ndarray, qw: np.ndarray) -> Tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    """
    Normalize quaternions for temporal continuity.

    Quaternions q and -q represent the same rotation. This function ensures
    continuity by flipping the sign when it would reduce the distance to the
    previous quaternion in the sequence. Starts with positive qw for the first sample.

    Args:
        qx, qy, qz, qw: Quaternion components as numpy arrays

    Returns:
        Tuple of (qx, qy, qz, qw) with temporal continuity
    """
    if len(qw) == 0:
        return qx, qy, qz, qw

    # Copy arrays to avoid modifying originals
    qx_out = qx.copy()
    qy_out = qy.copy()
    qz_out = qz.copy()
    qw_out = qw.copy()

    # Start with positive qw for first quaternion
    if qw_out[0] < 0:
        qx_out[0] = -qx_out[0]
        qy_out[0] = -qy_out[0]
        qz_out[0] = -qz_out[0]
        qw_out[0] = -qw_out[0]

    # Ensure continuity for subsequent quaternions
    for i in range(1, len(qw)):
        # Compute dot product with previous quaternion
        dot = (qx_out[i] * qx_out[i-1] + qy_out[i] * qy_out[i-1] +
               qz_out[i] * qz_out[i-1] + qw_out[i] * qw_out[i-1])

        # If dot product is negative, flip sign to get shorter path
        if dot < 0:
            qx_out[i] = -qx_out[i]
            qy_out[i] = -qy_out[i]
            qz_out[i] = -qz_out[i]
            qw_out[i] = -qw_out[i]

    return qx_out, qy_out, qz_out, qw_out


def quaternion_to_rotation_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """
    Convert quaternion to 3x3 rotation matrix.

    Args:
        qx, qy, qz, qw: Quaternion components

    Returns:
        3x3 rotation matrix
    """
    # Normalize quaternion
    norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Rotation matrix from quaternion
    R = np.array([
        [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
        [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
        [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
    ])
    return R


def vee_map(S: np.ndarray) -> np.ndarray:
    """
    Extract vector from skew-symmetric matrix (vee map).

    Args:
        S: 3x3 skew-symmetric matrix

    Returns:
        3-element vector [S32, S13, S21]
    """
    return np.array([S[2, 1], S[0, 2], S[1, 0]])


def compute_rotation_error(R: np.ndarray, R_d: np.ndarray) -> np.ndarray:
    """
    Compute the SO(3) rotation error vector.

    E_R = 1/2 * vee(R_d^T * R - R^T * R_d)

    This is the standard geometric control rotation error from Lee et al.

    Args:
        R: Actual rotation matrix (3x3)
        R_d: Desired rotation matrix (3x3)

    Returns:
        3-element rotation error vector
    """
    error_matrix = R_d.T @ R - R.T @ R_d
    return 0.5 * vee_map(error_matrix)


# ==============================================================================
# ROS Bag Reader
# ==============================================================================

class RosBagReader:
    """Read and parse telemetry data from ROS bags."""

    TOPIC_ODOMETRY = '/mavros/odometry/in'
    TOPIC_POSITION_SETPOINT = '/mavros/setpoint_raw/local'
    TOPIC_ATTITUDE_SETPOINT = '/mavros/setpoint_raw/attitude'

    def __init__(self, bag_path: str):
        """
        Initialize the ROS bag reader.

        Args:
            bag_path: Path to the ROS bag file (supports bz2 compression)
        """
        self.bag_path = bag_path
        self.bag = None
        self.start_time = None

        # Data containers
        self.odometry = OdometryData()
        self.position_setpoint = PositionSetpointData()
        self.attitude_setpoint = AttitudeSetpointData()

    def read(self, start_time: Optional[float] = None,
             end_time: Optional[float] = None) -> None:
        """
        Read data from the ROS bag.

        Args:
            start_time: Start time in seconds (relative to bag start)
            end_time: End time in seconds (relative to bag start)
        """
        print(f"Reading bag: {self.bag_path}")

        # Temporary lists for accumulating data
        odom_data = {'t': [], 'x': [], 'y': [], 'z': [],
                     'vx': [], 'vy': [], 'vz': [],
                     'qx': [], 'qy': [], 'qz': [], 'qw': []}
        pos_sp_data = {'t': [], 'x': [], 'y': [], 'z': [],
                       'vx': [], 'vy': [], 'vz': [],
                       'ax': [], 'ay': [], 'az': [],
                       'yaw': [], 'yaw_rate': []}
        att_sp_data = {'t': [], 'qx': [], 'qy': [], 'qz': [], 'qw': [],
                       'roll_rate': [], 'pitch_rate': [], 'yaw_rate': [],
                       'thrust': []}

        with rosbag.Bag(self.bag_path, 'r') as bag:
            # Get bag info
            info = bag.get_type_and_topic_info()
            print(f"Bag duration: {bag.get_end_time() - bag.get_start_time():.2f}s")
            print(f"Topics: {list(info.topics.keys())}")

            bag_start = bag.get_start_time()
            self.start_time = bag_start

            # Calculate absolute time bounds
            abs_start = bag_start + (start_time if start_time else 0)
            abs_end = bag.get_end_time() if end_time is None else bag_start + end_time

            # Read messages
            for topic, msg, t in bag.read_messages():
                t_sec = t.to_sec()

                # Skip if outside time bounds
                if t_sec < abs_start or t_sec > abs_end:
                    continue

                rel_time = t_sec - abs_start

                if topic == self.TOPIC_ODOMETRY:
                    self._parse_odometry(msg, rel_time, odom_data)
                elif topic == self.TOPIC_POSITION_SETPOINT:
                    self._parse_position_setpoint(msg, rel_time, pos_sp_data)
                elif topic == self.TOPIC_ATTITUDE_SETPOINT:
                    self._parse_attitude_setpoint(msg, rel_time, att_sp_data)

        # Convert to numpy arrays
        self._finalize_data(odom_data, pos_sp_data, att_sp_data)

        print(f"Loaded {len(self.odometry.timestamps)} odometry samples")
        print(f"Loaded {len(self.position_setpoint.timestamps)} position setpoint samples")
        print(f"Loaded {len(self.attitude_setpoint.timestamps)} attitude setpoint samples")

    def _parse_odometry(self, msg, t: float, data: Dict) -> None:
        """Parse nav_msgs/Odometry message."""
        data['t'].append(t)
        data['x'].append(msg.pose.pose.position.x)
        data['y'].append(msg.pose.pose.position.y)
        data['z'].append(msg.pose.pose.position.z)
        data['vx'].append(msg.twist.twist.linear.x)
        data['vy'].append(msg.twist.twist.linear.y)
        data['vz'].append(msg.twist.twist.linear.z)
        data['qx'].append(msg.pose.pose.orientation.x)
        data['qy'].append(msg.pose.pose.orientation.y)
        data['qz'].append(msg.pose.pose.orientation.z)
        data['qw'].append(msg.pose.pose.orientation.w)

    def _parse_position_setpoint(self, msg, t: float, data: Dict) -> None:
        """Parse mavros_msgs/PositionTarget message."""
        data['t'].append(t)
        data['x'].append(msg.position.x)
        data['y'].append(msg.position.y)
        data['z'].append(msg.position.z)
        data['vx'].append(msg.velocity.x)
        data['vy'].append(msg.velocity.y)
        data['vz'].append(msg.velocity.z)
        data['ax'].append(msg.acceleration_or_force.x)
        data['ay'].append(msg.acceleration_or_force.y)
        data['az'].append(msg.acceleration_or_force.z)
        data['yaw'].append(msg.yaw)
        data['yaw_rate'].append(msg.yaw_rate)

    def _parse_attitude_setpoint(self, msg, t: float, data: Dict) -> None:
        """Parse mavros_msgs/AttitudeTarget message."""
        data['t'].append(t)
        data['qx'].append(msg.orientation.x)
        data['qy'].append(msg.orientation.y)
        data['qz'].append(msg.orientation.z)
        data['qw'].append(msg.orientation.w)
        data['roll_rate'].append(msg.body_rate.x)
        data['pitch_rate'].append(msg.body_rate.y)
        data['yaw_rate'].append(msg.body_rate.z)
        data['thrust'].append(msg.thrust)

    def _finalize_data(self, odom_data: Dict, pos_sp_data: Dict,
                       att_sp_data: Dict) -> None:
        """Convert accumulated lists to numpy arrays and compute derived quantities."""
        # Odometry
        self.odometry.timestamps = np.array(odom_data['t'])
        self.odometry.x = np.array(odom_data['x'])
        self.odometry.y = np.array(odom_data['y'])
        self.odometry.z = np.array(odom_data['z'])
        self.odometry.vx = np.array(odom_data['vx'])
        self.odometry.vy = np.array(odom_data['vy'])
        self.odometry.vz = np.array(odom_data['vz'])
        self.odometry.qx = np.array(odom_data['qx'])
        self.odometry.qy = np.array(odom_data['qy'])
        self.odometry.qz = np.array(odom_data['qz'])
        self.odometry.qw = np.array(odom_data['qw'])

        # Compute Euler angles for odometry
        if len(self.odometry.qx) > 0:
            roll, pitch, yaw = quaternion_to_euler_vectorized(
                self.odometry.qx, self.odometry.qy,
                self.odometry.qz, self.odometry.qw
            )
            self.odometry.roll = roll
            self.odometry.pitch = pitch
            self.odometry.yaw = unwrap_angle(yaw)

        # Position setpoint
        self.position_setpoint.timestamps = np.array(pos_sp_data['t'])
        self.position_setpoint.x = np.array(pos_sp_data['x'])
        self.position_setpoint.y = np.array(pos_sp_data['y'])
        self.position_setpoint.z = np.array(pos_sp_data['z'])
        self.position_setpoint.vx = np.array(pos_sp_data['vx'])
        self.position_setpoint.vy = np.array(pos_sp_data['vy'])
        self.position_setpoint.vz = np.array(pos_sp_data['vz'])
        self.position_setpoint.ax = np.array(pos_sp_data['ax'])
        self.position_setpoint.ay = np.array(pos_sp_data['ay'])
        self.position_setpoint.az = np.array(pos_sp_data['az'])
        self.position_setpoint.yaw = unwrap_angle(np.array(pos_sp_data['yaw']))
        self.position_setpoint.yaw_rate = np.array(pos_sp_data['yaw_rate'])

        # Attitude setpoint
        self.attitude_setpoint.timestamps = np.array(att_sp_data['t'])
        self.attitude_setpoint.qx = np.array(att_sp_data['qx'])
        self.attitude_setpoint.qy = np.array(att_sp_data['qy'])
        self.attitude_setpoint.qz = np.array(att_sp_data['qz'])
        self.attitude_setpoint.qw = np.array(att_sp_data['qw'])
        self.attitude_setpoint.roll_rate = np.array(att_sp_data['roll_rate'])
        self.attitude_setpoint.pitch_rate = np.array(att_sp_data['pitch_rate'])
        self.attitude_setpoint.yaw_rate = np.array(att_sp_data['yaw_rate'])
        self.attitude_setpoint.thrust = np.array(att_sp_data['thrust'])

        # Compute Euler angles for attitude setpoint
        if len(self.attitude_setpoint.qx) > 0:
            roll, pitch, yaw = quaternion_to_euler_vectorized(
                self.attitude_setpoint.qx, self.attitude_setpoint.qy,
                self.attitude_setpoint.qz, self.attitude_setpoint.qw
            )
            self.attitude_setpoint.roll = roll
            self.attitude_setpoint.pitch = pitch
            self.attitude_setpoint.yaw = unwrap_angle(yaw)


# ==============================================================================
# Plotting Functions
# ==============================================================================

class TelemetryPlotter:
    """Generate publication-quality plots from telemetry data."""

    def __init__(self, reader: RosBagReader, output_dir: str = '.',
                 dpi: int = 300, format: str = 'pdf'):
        """
        Initialize the plotter.

        Args:
            reader: RosBagReader instance with loaded data
            output_dir: Directory to save figures
            dpi: Resolution for raster formats
            format: Output format (pdf, png, eps, svg)
        """
        self.reader = reader
        self.output_dir = output_dir
        self.dpi = dpi
        self.format = format

        os.makedirs(output_dir, exist_ok=True)
        setup_publication_style()

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

    def _format_axis(self, ax: plt.Axes, xlabel: str, ylabel: str,
                     title: Optional[str] = None) -> None:
        """Apply consistent formatting to axis."""
        ax.set_xlabel(xlabel)
        ax.set_ylabel(ylabel)
        if title:
            ax.set_title(title)
        self._add_minor_ticks(ax)
        ax.grid(True, which='major', alpha=0.5)
        ax.grid(True, which='minor', alpha=0.2)

    def plot_position_tracking(self) -> str:
        """
        Plot position tracking (actual vs setpoint) for x, y, z.

        Returns:
            Path to saved figure
        """
        fig, axes = plt.subplots(3, 1, figsize=(COLUMN_WIDTH_INCHES, 4.5),
                                  sharex=True)

        odom = self.reader.odometry
        sp = self.reader.position_setpoint

        labels = ['$x$', '$y$', '$z$']
        odom_data = [odom.x, odom.y, odom.z]
        sp_data = [sp.x, sp.y, sp.z]
        colors = [COLORS['x'], COLORS['y'], COLORS['z']]

        for ax, label, od, spd, color in zip(axes, labels, odom_data, sp_data, colors):
            # Plot setpoint first (behind actual)
            if len(sp.timestamps) > 0:
                ax.plot(sp.timestamps, spd,
                       linestyle=LINE_STYLES['setpoint'],
                       linewidth=LINE_WIDTHS['setpoint'],
                       color=color, alpha=0.7, label='Setpoint')

            # Plot actual
            if len(odom.timestamps) > 0:
                ax.plot(odom.timestamps, od,
                       linestyle=LINE_STYLES['actual'],
                       linewidth=LINE_WIDTHS['actual'],
                       color=color, label='Actual')

            ax.set_ylabel(f'{label} (m)')
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

        axes[-1].set_xlabel('Time (s)')

        # Single legend for all subplots
        handles, labels_leg = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels_leg, loc='upper right',
                   bbox_to_anchor=(0.98, 0.98), ncol=2, framealpha=0.9)

        fig.tight_layout()
        fig.subplots_adjust(top=0.92)

        return self._save_figure(fig, 'position_tracking')

    def plot_velocity_tracking(self) -> str:
        """Plot velocity tracking (actual vs setpoint) for vx, vy, vz."""
        fig, axes = plt.subplots(3, 1, figsize=(COLUMN_WIDTH_INCHES, 4.5),
                                  sharex=True)

        odom = self.reader.odometry
        sp = self.reader.position_setpoint

        labels = ['$v_x$', '$v_y$', '$v_z$']
        odom_data = [odom.vx, odom.vy, odom.vz]
        sp_data = [sp.vx, sp.vy, sp.vz]
        colors = [COLORS['x'], COLORS['y'], COLORS['z']]

        for ax, label, od, spd, color in zip(axes, labels, odom_data, sp_data, colors):
            if len(sp.timestamps) > 0:
                ax.plot(sp.timestamps, spd,
                       linestyle=LINE_STYLES['setpoint'],
                       linewidth=LINE_WIDTHS['setpoint'],
                       color=color, alpha=0.7, label='Setpoint')

            if len(odom.timestamps) > 0:
                ax.plot(odom.timestamps, od,
                       linestyle=LINE_STYLES['actual'],
                       linewidth=LINE_WIDTHS['actual'],
                       color=color, label='Actual')

            ax.set_ylabel(f'{label} (m/s)')
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

        axes[-1].set_xlabel('Time (s)')

        handles, labels_leg = axes[0].get_legend_handles_labels()
        fig.legend(handles, labels_leg, loc='upper right',
                   bbox_to_anchor=(0.98, 0.98), ncol=2, framealpha=0.9)

        fig.tight_layout()
        fig.subplots_adjust(top=0.92)

        return self._save_figure(fig, 'velocity_tracking')

    def plot_attitude_tracking(self) -> str:
        """Plot attitude tracking with qx, qy, qz and rotation error E_R."""
        fig, axes = plt.subplots(2, 1, figsize=(COLUMN_WIDTH_INCHES, 4.5),
                                  sharex=True, height_ratios=[1, 1])

        odom = self.reader.odometry
        att_sp = self.reader.attitude_setpoint

        # Normalize quaternions for temporal continuity
        if len(odom.timestamps) > 0:
            odom_qx, odom_qy, odom_qz, odom_qw = normalize_quaternion_continuous(
                odom.qx, odom.qy, odom.qz, odom.qw
            )
        else:
            odom_qx, odom_qy, odom_qz, odom_qw = [], [], [], []

        if len(att_sp.timestamps) > 0:
            sp_qx, sp_qy, sp_qz, sp_qw = normalize_quaternion_continuous(
                att_sp.qx, att_sp.qy, att_sp.qz, att_sp.qw
            )
        else:
            sp_qx, sp_qy, sp_qz, sp_qw = [], [], [], []

        # ---- Top subplot: Quaternion components ----
        ax_quat = axes[0]
        labels = ['$q_x$', '$q_y$', '$q_z$']
        odom_data = [odom_qx, odom_qy, odom_qz]
        sp_data = [sp_qx, sp_qy, sp_qz]
        colors = [COLORS['x'], COLORS['y'], COLORS['z']]

        # Plot setpoints first (behind actual)
        for label, spd, color in zip(labels, sp_data, colors):
            if len(att_sp.timestamps) > 0 and len(spd) > 0:
                ax_quat.plot(att_sp.timestamps, spd,
                       linestyle=LINE_STYLES['setpoint'],
                       linewidth=0.6,
                       color=color, alpha=0.5)

        # Plot actual values
        for label, od, color in zip(labels, odom_data, colors):
            if len(odom.timestamps) > 0 and len(od) > 0:
                ax_quat.plot(odom.timestamps, od,
                       linestyle=LINE_STYLES['actual'],
                       linewidth=0.8,
                       color=color, label=label)

        ax_quat.set_ylabel('Quaternion')
        ax_quat.set_ylim(-1.1, 1.1)
        ax_quat.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
        self._add_minor_ticks(ax_quat)
        ax_quat.grid(True, which='major', alpha=0.5)
        ax_quat.grid(True, which='minor', alpha=0.2)
        ax_quat.legend(loc='upper right', ncol=3, fontsize=7, framealpha=0.9)

        # ---- Bottom subplot: Rotation error E_R (norm) ----
        ax_err = axes[1]

        # Compute rotation error if we have both actual and setpoint data
        if len(odom.timestamps) > 0 and len(att_sp.timestamps) > 0:
            # Interpolate setpoint quaternions to odometry timestamps
            sp_qx_interp = np.interp(odom.timestamps, att_sp.timestamps, sp_qx)
            sp_qy_interp = np.interp(odom.timestamps, att_sp.timestamps, sp_qy)
            sp_qz_interp = np.interp(odom.timestamps, att_sp.timestamps, sp_qz)
            sp_qw_interp = np.interp(odom.timestamps, att_sp.timestamps, sp_qw)

            # Compute rotation error norm at each timestep
            e_R_norm = np.zeros(len(odom.timestamps))

            for i in range(len(odom.timestamps)):
                # Actual rotation matrix
                R = quaternion_to_rotation_matrix(
                    odom_qx[i], odom_qy[i], odom_qz[i], odom_qw[i]
                )
                # Desired rotation matrix
                R_d = quaternion_to_rotation_matrix(
                    sp_qx_interp[i], sp_qy_interp[i], sp_qz_interp[i], sp_qw_interp[i]
                )
                # Compute error norm: ||e_R|| = ||1/2 * vee(R_d^T R - R^T R_d)||
                e_R = compute_rotation_error(R, R_d)
                e_R_norm[i] = np.linalg.norm(e_R)

            # Plot rotation error norm
            ax_err.plot(odom.timestamps, e_R_norm,
                       linestyle='-', linewidth=0.8,
                       color=COLORS['error'])

            # Compute and display RMSE
            rmse = np.sqrt(np.mean(e_R_norm**2))
            max_err = np.max(e_R_norm)
            ax_err.annotate(f'RMSE: {rmse:.4f}\nMax: {max_err:.4f}',
                           xy=(0.98, 0.95), xycoords='axes fraction',
                           ha='right', va='top', fontsize=7,
                           bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        ax_err.set_ylabel(r'$\|e_R\|$')
        ax_err.set_xlabel('Time (s)')
        ax_err.axhline(y=0, color='black', linewidth=0.3, linestyle='-', alpha=0.3)
        self._add_minor_ticks(ax_err)
        ax_err.grid(True, which='major', alpha=0.5)
        ax_err.grid(True, which='minor', alpha=0.2)

        fig.tight_layout()

        return self._save_figure(fig, 'attitude_tracking')

    def plot_position_error(self) -> str:
        """Plot position tracking error over time."""
        fig, axes = plt.subplots(4, 1, figsize=(COLUMN_WIDTH_INCHES, 5.5),
                                  sharex=True)

        odom = self.reader.odometry
        sp = self.reader.position_setpoint

        if len(odom.timestamps) == 0 or len(sp.timestamps) == 0:
            print("Warning: Insufficient data for position error plot")
            plt.close(fig)
            return ""

        # Interpolate setpoint to odometry timestamps
        x_sp_interp = np.interp(odom.timestamps, sp.timestamps, sp.x)
        y_sp_interp = np.interp(odom.timestamps, sp.timestamps, sp.y)
        z_sp_interp = np.interp(odom.timestamps, sp.timestamps, sp.z)

        # Compute errors
        ex = odom.x - x_sp_interp
        ey = odom.y - y_sp_interp
        ez = odom.z - z_sp_interp
        e_norm = np.sqrt(ex**2 + ey**2 + ez**2)

        errors = [ex, ey, ez, e_norm]
        labels = ['$e_x$', '$e_y$', '$e_z$', r'$\|e\|$']
        colors = [COLORS['x'], COLORS['y'], COLORS['z'], COLORS['error']]

        for ax, err, label, color in zip(axes, errors, labels, colors):
            ax.plot(odom.timestamps, err, color=color, linewidth=1.0)
            ax.axhline(y=0, color='black', linewidth=0.5, linestyle='-', alpha=0.3)
            ax.set_ylabel(f'{label} (m)')
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

        axes[-1].set_xlabel('Time (s)')

        # Add statistics annotation
        rmse = np.sqrt(np.mean(e_norm**2))
        max_err = np.max(np.abs(e_norm))
        stats_text = f'RMSE: {rmse*100:.2f} cm\nMax: {max_err*100:.2f} cm'
        axes[-1].annotate(stats_text, xy=(0.98, 0.95), xycoords='axes fraction',
                          ha='right', va='top', fontsize=7,
                          bbox=dict(boxstyle='round', facecolor='white', alpha=0.8))

        fig.tight_layout()

        return self._save_figure(fig, 'position_error')

    def plot_3d_trajectory(self, quiver_interval: float = 10.0) -> str:
        """
        Plot 3D trajectory (actual and setpoint) with orientation quivers.

        Args:
            quiver_interval: Time interval in seconds between orientation quivers (default: 10s)
        """
        fig = plt.figure(figsize=(COLUMN_WIDTH_INCHES, COLUMN_WIDTH_INCHES))
        ax = fig.add_subplot(111, projection='3d')

        odom = self.reader.odometry
        sp = self.reader.position_setpoint

        # Plot setpoint trajectory
        if len(sp.timestamps) > 0:
            ax.plot(sp.x, sp.y, sp.z,
                   linestyle=LINE_STYLES['setpoint'],
                   linewidth=LINE_WIDTHS['setpoint'],
                   color=COLORS['setpoint'], alpha=0.7, label='Setpoint')

        # Plot actual trajectory
        if len(odom.timestamps) > 0:
            ax.plot(odom.x, odom.y, odom.z,
                   linestyle=LINE_STYLES['actual'],
                   linewidth=LINE_WIDTHS['actual'],
                   color=COLORS['actual'], label='Actual')

            # Mark start and end points
            ax.scatter([odom.x[0]], [odom.y[0]], [odom.z[0]],
                      marker='o', s=30, color='green', label='Start', zorder=5)
            ax.scatter([odom.x[-1]], [odom.y[-1]], [odom.z[-1]],
                      marker='s', s=30, color='red', label='End', zorder=5)

            # Add orientation quivers every quiver_interval seconds
            # Normalize quaternions for consistency
            qx_norm, qy_norm, qz_norm, qw_norm = normalize_quaternion_continuous(
                odom.qx, odom.qy, odom.qz, odom.qw
            )

            # Calculate quiver length based on trajectory scale
            max_range = max(
                np.ptp(odom.x) if len(odom.x) > 0 else 1,
                np.ptp(odom.y) if len(odom.y) > 0 else 1,
                np.ptp(odom.z) if len(odom.z) > 0 else 1
            )
            quiver_length = max_range * 0.15  # 15% of trajectory scale

            # Find indices at regular time intervals
            t_start = odom.timestamps[0]
            t_end = odom.timestamps[-1]
            quiver_times = np.arange(t_start, t_end + quiver_interval, quiver_interval)

            for t_quiver in quiver_times:
                # Find closest index to this time
                idx = np.argmin(np.abs(odom.timestamps - t_quiver))

                # Get position and quaternion at this index
                pos = np.array([odom.x[idx], odom.y[idx], odom.z[idx]])
                quat = (qx_norm[idx], qy_norm[idx], qz_norm[idx], qw_norm[idx])

                # Convert quaternion to rotation matrix
                R = quaternion_to_rotation_matrix(*quat)

                # Get body frame axes (columns of rotation matrix)
                x_axis = R[:, 0] * quiver_length  # Red - X axis (forward)
                y_axis = R[:, 1] * quiver_length  # Green - Y axis (left)
                z_axis = R[:, 2] * quiver_length  # Blue - Z axis (up)

                # Draw RGB quivers for orientation
                ax.quiver(pos[0], pos[1], pos[2],
                         x_axis[0], x_axis[1], x_axis[2],
                         color='red', arrow_length_ratio=0.2, linewidth=1.2)
                ax.quiver(pos[0], pos[1], pos[2],
                         y_axis[0], y_axis[1], y_axis[2],
                         color='green', arrow_length_ratio=0.2, linewidth=1.2)
                ax.quiver(pos[0], pos[1], pos[2],
                         z_axis[0], z_axis[1], z_axis[2],
                         color='blue', arrow_length_ratio=0.2, linewidth=1.2)

        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        ax.set_zlabel('$z$ (m)')
        ax.legend(loc='upper left', fontsize=7)

        # Equal aspect ratio
        max_range = max(
            np.ptp(odom.x) if len(odom.x) > 0 else 1,
            np.ptp(odom.y) if len(odom.y) > 0 else 1,
            np.ptp(odom.z) if len(odom.z) > 0 else 1
        ) / 2

        if len(odom.x) > 0:
            mid_x = (np.max(odom.x) + np.min(odom.x)) / 2
            mid_y = (np.max(odom.y) + np.min(odom.y)) / 2
            mid_z = (np.max(odom.z) + np.min(odom.z)) / 2
            ax.set_xlim(mid_x - max_range, mid_x + max_range)
            ax.set_ylim(mid_y - max_range, mid_y + max_range)
            ax.set_zlim(mid_z - max_range, mid_z + max_range)

        fig.tight_layout()

        return self._save_figure(fig, '3d_trajectory')

    def plot_xy_trajectory(self) -> str:
        """Plot XY (top-down) trajectory view."""
        fig, ax = plt.subplots(figsize=(COLUMN_WIDTH_INCHES, COLUMN_WIDTH_INCHES))

        odom = self.reader.odometry
        sp = self.reader.position_setpoint

        # Plot setpoint
        if len(sp.timestamps) > 0:
            ax.plot(sp.x, sp.y,
                   linestyle=LINE_STYLES['setpoint'],
                   linewidth=LINE_WIDTHS['setpoint'],
                   color=COLORS['setpoint'], alpha=0.7, label='Setpoint')

        # Plot actual
        if len(odom.timestamps) > 0:
            ax.plot(odom.x, odom.y,
                   linestyle=LINE_STYLES['actual'],
                   linewidth=LINE_WIDTHS['actual'],
                   color=COLORS['actual'], label='Actual')

            # Start/end markers
            ax.scatter([odom.x[0]], [odom.y[0]], marker='o', s=40,
                      color='green', label='Start', zorder=5)
            ax.scatter([odom.x[-1]], [odom.y[-1]], marker='s', s=40,
                      color='red', label='End', zorder=5)

        ax.set_xlabel('$x$ (m)')
        ax.set_ylabel('$y$ (m)')
        ax.set_aspect('equal', adjustable='box')
        ax.legend(loc='best', fontsize=7)
        self._add_minor_ticks(ax)
        ax.grid(True, which='major', alpha=0.5)
        ax.grid(True, which='minor', alpha=0.2)

        fig.tight_layout()

        return self._save_figure(fig, 'xy_trajectory')

    def plot_combined_state(self) -> str:
        """
        Create a combined figure with position, velocity, and quaternion attitude.
        Full-width figure suitable for journal papers.
        """
        fig = plt.figure(figsize=(FULL_WIDTH_INCHES, 5.5))

        # Create grid: 3 columns (pos, vel, quat) x 3 rows (x/vx/qx, y/vy/qy, z/vz/qz)
        gs = fig.add_gridspec(3, 3, hspace=0.1, wspace=0.25)

        odom = self.reader.odometry
        pos_sp = self.reader.position_setpoint
        att_sp = self.reader.attitude_setpoint

        # Normalize quaternions for temporal continuity
        if len(odom.timestamps) > 0:
            odom_qx, odom_qy, odom_qz, odom_qw = normalize_quaternion_continuous(
                odom.qx, odom.qy, odom.qz, odom.qw
            )
        else:
            odom_qx, odom_qy, odom_qz, odom_qw = [], [], [], []

        if len(att_sp.timestamps) > 0:
            sp_qx, sp_qy, sp_qz, sp_qw = normalize_quaternion_continuous(
                att_sp.qx, att_sp.qy, att_sp.qz, att_sp.qw
            )
        else:
            sp_qx, sp_qy, sp_qz, sp_qw = [], [], [], []

        # Column titles
        col_titles = ['Position', 'Velocity', 'Quaternion']

        # Position plots (column 0)
        pos_labels = ['$x$ (m)', '$y$ (m)', '$z$ (m)']
        pos_odom = [odom.x, odom.y, odom.z]
        pos_sp_data = [pos_sp.x, pos_sp.y, pos_sp.z]
        pos_colors = [COLORS['x'], COLORS['y'], COLORS['z']]

        # Velocity plots (column 1)
        vel_labels = ['$v_x$ (m/s)', '$v_y$ (m/s)', '$v_z$ (m/s)']
        vel_odom = [odom.vx, odom.vy, odom.vz]
        vel_sp_data = [pos_sp.vx, pos_sp.vy, pos_sp.vz]

        # Quaternion plots (column 2) - only qx, qy, qz
        quat_labels = ['$q_x$', '$q_y$', '$q_z$']
        quat_odom = [odom_qx, odom_qy, odom_qz]
        quat_sp_data = [sp_qx, sp_qy, sp_qz]
        quat_colors = [COLORS['x'], COLORS['y'], COLORS['z']]

        axes = []
        for row in range(3):
            row_axes = []
            for col in range(3):
                ax = fig.add_subplot(gs[row, col])
                row_axes.append(ax)

                if col == 0:  # Position
                    if len(pos_sp.timestamps) > 0:
                        ax.plot(pos_sp.timestamps, pos_sp_data[row],
                               linestyle='--', linewidth=0.6, color=pos_colors[row],
                               alpha=0.7, label='Setpoint')
                    if len(odom.timestamps) > 0:
                        ax.plot(odom.timestamps, pos_odom[row],
                               linestyle='-', linewidth=0.8, color=pos_colors[row],
                               label='Actual')
                    ax.set_ylabel(pos_labels[row])

                elif col == 1:  # Velocity
                    if len(pos_sp.timestamps) > 0:
                        ax.plot(pos_sp.timestamps, vel_sp_data[row],
                               linestyle='--', linewidth=0.6, color=pos_colors[row],
                               alpha=0.7)
                    if len(odom.timestamps) > 0:
                        ax.plot(odom.timestamps, vel_odom[row],
                               linestyle='-', linewidth=0.8, color=pos_colors[row])
                    ax.set_ylabel(vel_labels[row])

                else:  # Quaternion
                    if len(att_sp.timestamps) > 0 and len(quat_sp_data[row]) > 0:
                        ax.plot(att_sp.timestamps, quat_sp_data[row],
                               linestyle='--', linewidth=0.6, color=quat_colors[row],
                               alpha=0.5)
                    if len(odom.timestamps) > 0 and len(quat_odom[row]) > 0:
                        ax.plot(odom.timestamps, quat_odom[row],
                               linestyle='-', linewidth=0.8, color=quat_colors[row])
                    ax.set_ylabel(quat_labels[row])
                    ax.set_ylim(-1.1, 1.1)

                # Only show x-axis label on bottom row
                if row == 2:
                    ax.set_xlabel('Time (s)')
                else:
                    ax.tick_params(labelbottom=False)

                # Add title to top row
                if row == 0:
                    ax.set_title(col_titles[col], fontsize=10)

                self._add_minor_ticks(ax)
                ax.grid(True, which='major', alpha=0.4)
                ax.grid(True, which='minor', alpha=0.15)

            axes.append(row_axes)

        # Add legend
        handles, labels_leg = axes[0][0].get_legend_handles_labels()
        fig.legend(handles, labels_leg, loc='upper center',
                   bbox_to_anchor=(0.5, 1.02), ncol=2, framealpha=0.9)

        fig.tight_layout()
        fig.subplots_adjust(top=0.92)

        return self._save_figure(fig, 'combined_state')

    def plot_body_rates(self) -> str:
        """Plot body rate setpoints and tracking."""
        fig, axes = plt.subplots(3, 1, figsize=(COLUMN_WIDTH_INCHES, 4.5),
                                  sharex=True)

        att_sp = self.reader.attitude_setpoint

        labels = [r'$p$ (Roll rate)', r'$q$ (Pitch rate)', r'$r$ (Yaw rate)']
        rates = [np.rad2deg(att_sp.roll_rate),
                 np.rad2deg(att_sp.pitch_rate),
                 np.rad2deg(att_sp.yaw_rate)]
        colors = [COLORS['roll'], COLORS['pitch'], COLORS['yaw']]

        for ax, label, rate, color in zip(axes, labels, rates, colors):
            if len(att_sp.timestamps) > 0:
                ax.plot(att_sp.timestamps, rate, color=color, linewidth=1.0)
            ax.set_ylabel(f'{label} (deg/s)')
            ax.axhline(y=0, color='black', linewidth=0.5, linestyle='-', alpha=0.3)
            self._add_minor_ticks(ax)
            ax.grid(True, which='major', alpha=0.5)
            ax.grid(True, which='minor', alpha=0.2)

        axes[-1].set_xlabel('Time (s)')

        fig.tight_layout()

        return self._save_figure(fig, 'body_rates')

    def plot_all(self) -> List[str]:
        """Generate all standard plots."""
        paths = []

        print("\nGenerating plots...")
        paths.append(self.plot_position_tracking())
        paths.append(self.plot_velocity_tracking())
        paths.append(self.plot_attitude_tracking())
        paths.append(self.plot_position_error())
        paths.append(self.plot_3d_trajectory())
        paths.append(self.plot_xy_trajectory())
        paths.append(self.plot_combined_state())
        paths.append(self.plot_body_rates())

        # Filter empty paths
        paths = [p for p in paths if p]

        print(f"\nGenerated {len(paths)} figures in {self.output_dir}/")
        return paths


# ==============================================================================
# Main Entry Point
# ==============================================================================

def main():
    parser = argparse.ArgumentParser(
        description='Generate publication-quality plots from ROS bag telemetry data.',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  %(prog)s flight_data.bag
  %(prog)s flight_data.bag --output figures/
  %(prog)s flight_data.bag --start 10 --end 60 --dpi 600
  %(prog)s flight_data.bag --format png --dpi 300
        """
    )

    parser.add_argument('bag_file', help='Path to ROS bag file (supports bz2 compression)')
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
    parser.add_argument('--plots', '-p', nargs='+',
                        choices=['position', 'velocity', 'attitude', 'error',
                                '3d', 'xy', 'combined', 'rates', 'all'],
                        default=['all'],
                        help='Plots to generate (default: all)')

    args = parser.parse_args()

    # Check bag file exists
    if not os.path.exists(args.bag_file):
        print(f"Error: Bag file not found: {args.bag_file}")
        sys.exit(1)

    # Read bag
    reader = RosBagReader(args.bag_file)
    reader.read(start_time=args.start, end_time=args.end)

    # Create plotter
    plotter = TelemetryPlotter(reader, output_dir=args.output,
                                dpi=args.dpi, format=args.format)

    # Generate plots
    if 'all' in args.plots:
        plotter.plot_all()
    else:
        if 'position' in args.plots:
            plotter.plot_position_tracking()
        if 'velocity' in args.plots:
            plotter.plot_velocity_tracking()
        if 'attitude' in args.plots:
            plotter.plot_attitude_tracking()
        if 'error' in args.plots:
            plotter.plot_position_error()
        if '3d' in args.plots:
            plotter.plot_3d_trajectory()
        if 'xy' in args.plots:
            plotter.plot_xy_trajectory()
        if 'combined' in args.plots:
            plotter.plot_combined_state()
        if 'rates' in args.plots:
            plotter.plot_body_rates()


if __name__ == '__main__':
    main()
