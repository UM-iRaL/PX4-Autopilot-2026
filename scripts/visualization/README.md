# ROS Bag Telemetry Visualizer

Publication-quality plotting tool for MAVROS telemetry data from ROS bags.

## Features

- Reads bz2-compressed ROS bags (`rosbag record -j`)
- Generates IEEE/conference-ready figures
- Colorblind-friendly color palette
- Vector PDF output (also supports PNG, EPS, SVG)
- Automatic RMSE and error statistics

## Requirements

```bash
# ROS environment (for rosbag Python package)
source /opt/ros/noetic/setup.bash

# Python packages (already in PX4 requirements)
pip install matplotlib numpy
```

## Quick Start

```bash
# Generate all plots
python scripts/visualization/rosbag_plotter.py flight_data.bag

# Output to specific directory
python scripts/visualization/rosbag_plotter.py flight_data.bag -o figures/
```

## Command Line Options

| Option | Description | Default |
|--------|-------------|---------|
| `bag_file` | Path to ROS bag file | (required) |
| `-o, --output` | Output directory | `./figures` |
| `-s, --start` | Start time in seconds | Bag start |
| `-e, --end` | End time in seconds | Bag end |
| `--dpi` | Resolution for raster formats | 300 |
| `-f, --format` | Output format: pdf, png, eps, svg | pdf |
| `-p, --plots` | Specific plots to generate | all |

## Examples

### Basic Usage

```bash
# All plots as PDF (recommended for papers)
python scripts/visualization/rosbag_plotter.py 2024-01-15-flight.bag

# High-res PNG for presentations
python scripts/visualization/rosbag_plotter.py flight.bag --format png --dpi 600
```

### Time Cropping

```bash
# Skip first 10 seconds, stop at 60 seconds
python scripts/visualization/rosbag_plotter.py flight.bag --start 10 --end 60

# First 30 seconds only
python scripts/visualization/rosbag_plotter.py flight.bag --end 30
```

### Selective Plots

```bash
# Only position and attitude
python scripts/visualization/rosbag_plotter.py flight.bag --plots position attitude

# Only 3D trajectory
python scripts/visualization/rosbag_plotter.py flight.bag --plots 3d xy
```

Available plot options: `position`, `velocity`, `attitude`, `error`, `3d`, `xy`, `combined`, `rates`, `all`

## Generated Figures

| File | Description | Size |
|------|-------------|------|
| `position_tracking.pdf` | X, Y, Z position vs setpoint | Column width |
| `velocity_tracking.pdf` | Velocity components vs setpoint | Column width |
| `attitude_tracking.pdf` | Roll, pitch, yaw vs setpoint | Column width |
| `position_error.pdf` | Tracking error with RMSE stats | Column width |
| `3d_trajectory.pdf` | 3D trajectory visualization | Square |
| `xy_trajectory.pdf` | Top-down trajectory view | Square |
| `combined_state.pdf` | Position + velocity + attitude | Full width |
| `body_rates.pdf` | Angular rate setpoints | Column width |

## ROS Topics Used

| Topic | Message Type | Data |
|-------|--------------|------|
| `/mavros/odometry/in` | `nav_msgs/Odometry` | EKF position, velocity, attitude |
| `/mavros/setpoint_raw/local` | `mavros_msgs/PositionTarget` | Position/velocity setpoints |
| `/mavros/setpoint_raw/attitude` | `mavros_msgs/AttitudeTarget` | Attitude/rate setpoints |

## Recording Bags

To record the necessary topics:

```bash
rosbag record -j \
    /mavros/odometry/in \
    /mavros/setpoint_raw/local \
    /mavros/setpoint_raw/attitude \
    -O flight_data
```

The `-j` flag enables bz2 compression, which this tool handles automatically.

## Python API

You can also use the module programmatically:

```python
from scripts.visualization import RosBagReader, TelemetryPlotter

# Read bag
reader = RosBagReader('flight_data.bag')
reader.read(start_time=10, end_time=60)

# Access data directly
print(f"Flight duration: {reader.odometry.timestamps[-1]:.1f}s")
print(f"Max altitude: {reader.odometry.z.max():.2f}m")

# Generate specific plots
plotter = TelemetryPlotter(reader, output_dir='figures/', dpi=300)
plotter.plot_position_tracking()
plotter.plot_3d_trajectory()

# Or generate all
plotter.plot_all()
```

## Customization

### Figure Sizes

The script uses IEEE standard sizes:
- **Column width**: 3.5 inches (single column figures)
- **Full width**: 7.16 inches (spanning both columns)

To modify, edit the constants at the top of `rosbag_plotter.py`:

```python
COLUMN_WIDTH_INCHES = 3.5
FULL_WIDTH_INCHES = 7.16
```

### Colors

The default colorblind-friendly palette (Wong 2011) can be customized:

```python
COLORS = {
    'actual': '#0072B2',      # Blue
    'setpoint': '#D55E00',    # Vermillion
    'x': '#0072B2',
    'y': '#D55E00',
    'z': '#009E73',
    # ...
}
```

### Fonts

For LaTeX-rendered labels (requires LaTeX installation):

```python
plt.rcParams['text.usetex'] = True
```

## Troubleshooting

### "No module named rosbag"

Source ROS before running:
```bash
source /opt/ros/noetic/setup.bash
```

### Empty plots

Check that your bag contains the expected topics:
```bash
rosbag info flight_data.bag
```

### Time alignment issues

The script interpolates setpoints to odometry timestamps. If setpoints are published at a much lower rate, consider increasing the setpoint publish rate during recording.

## Citation

If you use this tool in your research, please cite the IRAL lab's morphable drone work.
