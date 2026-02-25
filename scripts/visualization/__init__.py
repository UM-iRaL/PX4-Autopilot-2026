"""
Visualization module for ROS bag telemetry data.

This module provides tools for generating publication-quality plots
from ROS bag files containing MAVROS telemetry data.
"""

from .rosbag_plotter import (
    RosBagReader,
    TelemetryPlotter,
    OdometryData,
    PositionSetpointData,
    AttitudeSetpointData,
    setup_publication_style,
)

__all__ = [
    'RosBagReader',
    'TelemetryPlotter',
    'OdometryData',
    'PositionSetpointData',
    'AttitudeSetpointData',
    'setup_publication_style',
]
