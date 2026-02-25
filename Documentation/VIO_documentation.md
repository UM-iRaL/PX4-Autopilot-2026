# VIO (Visual Inertial Odometry) Setup Documentation

## Auto-start ROS Bridge on Boot

This document explains how to automatically launch the PX4 RealSense bridge with MAVROS when the Jetson boots up.

### Systemd Service Setup

Create the systemd service file at `/etc/systemd/system/ros-bridge.service`:

```bash
sudo bash -c 'cat > /etc/systemd/system/ros-bridge.service << '\''EOF'\''
[Unit]
Description=ROS PX4 Realsense Bridge
After=network-online.target
Wants=network-online.target

[Service]
Type=simple
User=morphable-jetson
Environment="ROS_HOME=/home/morphable-jetson/.ros"
Environment="ROS_LOG_DIR=/home/morphable-jetson/.ros/log"
WorkingDirectory=/home/morphable-jetson
ExecStartPre=/bin/sleep 10
ExecStart=/bin/bash -c "source /opt/ros/noetic/setup.bash && source /home/morphable-jetson/catkin_ws/devel/setup.bash && roslaunch px4_realsense_bridge bridge_mavros.launch"
Restart=on-failure
RestartSec=10

[Install]
WantedBy=multi-user.target
EOF'
```

### Enable and Start the Service

```bash
# Reload systemd to recognize the new service
sudo systemctl daemon-reload

# Enable the service to start on boot
sudo systemctl enable ros-bridge.service

# Start the service now
sudo systemctl start ros-bridge.service
```

### Service Management Commands

#### Check Service Status
```bash
sudo systemctl status ros-bridge.service
```

#### View Service Logs
```bash
# View recent logs
sudo journalctl -u ros-bridge.service

# Follow logs in real-time
sudo journalctl -u ros-bridge.service -f

# View logs from current boot
sudo journalctl -u ros-bridge.service -b
```

#### Stop the Service
```bash
sudo systemctl stop ros-bridge.service
```

#### Restart the Service
```bash
sudo systemctl restart ros-bridge.service
```

#### Disable Auto-start on Boot
```bash
sudo systemctl disable ros-bridge.service
```

### Troubleshooting

If the service fails to start:

1. Check the service status for error messages:
   ```bash
   sudo systemctl status ros-bridge.service
   ```

2. View detailed logs:
   ```bash
   sudo journalctl -u ros-bridge.service -n 50
   ```

3. Verify ROS paths are correct:
   - ROS installation: `/opt/ros/noetic/setup.bash`
   - Catkin workspace: `/home/morphable-jetson/catkin_ws/devel/setup.bash`

4. Test the launch command manually:
   ```bash
   source /opt/ros/noetic/setup.bash
   source /home/morphable-jetson/catkin_ws/devel/setup.bash
   roslaunch px4_realsense_bridge bridge_mavros.launch
   ```

5. Check file permissions:
   ```bash
   cat /etc/systemd/system/ros-bridge.service
   ```

## Manual Launch

To manually run the bridge without the service:

```bash
roslaunch px4_realsense_bridge bridge_mavros.launch
```
