"""
Wait trajectory implementation with dynamic time scaling.

This module provides the WaitTrajectory class which supports trajectory generation
from either equations or quintic polynomial interpolation between waypoints.
It includes dynamic time scaling to help the robot keep up with the trajectory.
"""

import numpy as np
import rospy


class WaitTrajectory:
    """
    Trajectory generator with dynamic time scaling support.

    Supports three initialization modes:
    - 'equations': Trajectory defined by mathematical functions
    - 'quintic_polynomial': Trajectory interpolated through waypoints using quintic splines
    - 'piecewise_linear': Trajectory using linear interpolation between waypoints
    """

    def __init__(self, init_type, time_scaling_gain=0.5, **kwargs):
        """
        Initialize wait trajectory.

        Args:
            init_type: Either 'equations', 'quintic_polynomial', or 'piecewise_linear'
            time_scaling_gain: Proportional gain for dynamic time scaling (default: 0.5)
            **kwargs: Additional parameters depending on init_type
                For 'equations': pos_func, vel_func, acc_func, angle_func, omega_func, alpha_func
                For 'quintic_polynomial': waypoints (list of tuples)
                For 'piecewise_linear': waypoints (list of tuples)
        """
        self.init_type = init_type
        self.traj = None
        self.fig = None
        self.ax = None

        # State for Dynamic Time Scaling
        self.time_scaling_gain = time_scaling_gain  # Kp gain
        self.virtual_t = 0.0  # The planner's internal, adjusted time
        self.last_real_t = 0.0  # Last real time stamp seen by get()

        # Track current waypoint index for L1 parameter changes
        self.current_waypoint_index = 0
        self.last_waypoint_index = -1  # Used to detect waypoint transitions

        if init_type == 'equations':
            self.pos_func = kwargs['pos_func']
            self.vel_func = kwargs['vel_func']
            self.acc_func = kwargs.get('acc_func', None)
            self.angle_func = kwargs['angle_func']
            self.omega_func = kwargs['omega_func']
            self.alpha_func = kwargs.get('alpha_func', None)
        elif init_type == 'quintic_polynomial':
            self.waypoints = kwargs['waypoints']
            # Get the start time from the first waypoint (index 6 is time, -1 is l1_params)
            if self.waypoints:
                self.virtual_t = self.waypoints[0][6]  # Time is at index 6
                self.last_real_t = self.waypoints[0][6]
            self._compute_quintic_coeffs()
        elif init_type == 'piecewise_linear':
            self.waypoints = kwargs['waypoints']
            # Get the start time from the first waypoint (index 6 is time, -1 is l1_params)
            if self.waypoints:
                self.virtual_t = self.waypoints[0][6]  # Time is at index 6
                self.last_real_t = self.waypoints[0][6]
            # No need to compute coefficients for piecewise linear
        else:
            raise ValueError('Unknown initialization type')

    def _compute_quintic_coeffs(self):
        """Compute quintic polynomial coefficients for interpolation between waypoints.

        Uses normalized time tau in [0, 1] per segment for numerical stability.
        With absolute time, entries reach t^5 (e.g. 28^5 = 17M) causing an
        ill-conditioned Vandermonde matrix and corrupted derivatives. Normalized
        time keeps all matrix entries bounded by 20.

        The polynomial p(tau) relates to real time t via tau = (t - t0) / h:
            dp/dt = dp/dtau / h,   d2p/dt2 = d2p/dtau2 / h^2
        So boundary conditions scale: vel_tau = vel * h, acc_tau = acc * h^2.

        Decouple flags: When a waypoint has decouple.acceleration=True, the
        polynomial is fitted with zero acceleration BCs so position/velocity
        stay smooth. The raw acceleration is stored separately and injected
        at evaluation time. Similarly for decouple.velocity.
        """
        # Each waypoint: (pos, vel, acc, angles, omega, alpha, time, l1_params, decouple_flags)
        self.coeffs_pos = []
        self.coeffs_ang = []
        self.segment_decouple = []  # Per-segment decouple info

        # Normalized-time constraint matrix (same for every segment)
        # tau=0: p(0), p'(0), p''(0);  tau=1: p(1), p'(1), p''(1)
        M = np.array([
            [1, 0, 0,  0,  0,  0],
            [0, 1, 0,  0,  0,  0],
            [0, 0, 2,  0,  0,  0],
            [1, 1, 1,  1,  1,  1],
            [0, 1, 2,  3,  4,  5],
            [0, 0, 2,  6, 12, 20]
        ], dtype=float)

        for i in range(len(self.waypoints) - 1):
            p0, v0, a0, th0, w0, al0, t0, *extras0 = self.waypoints[i]
            p1, v1, a1, th1, w1, al1, t1, *extras1 = self.waypoints[i + 1]
            h = t1 - t0
            if h < 1e-10:
                h = 1e-10  # Guard against zero-length segments

            # Extract decouple flags (index 1 in extras = decouple_flags after l1_params)
            df0 = extras0[1] if len(extras0) > 1 and extras0[1] is not None else {}
            df1 = extras1[1] if len(extras1) > 1 and extras1[1] is not None else {}

            decouple_acc = df0.get('acceleration', False) or df1.get('acceleration', False)
            decouple_vel = df0.get('velocity', False) or df1.get('velocity', False)

            # For polynomial fitting, use zero where decoupled
            fit_a0 = np.zeros(3) if decouple_acc else a0
            fit_a1 = np.zeros(3) if decouple_acc else a1
            fit_v0 = np.zeros(3) if decouple_vel else v0
            fit_v1 = np.zeros(3) if decouple_vel else v1
            # If velocity is decoupled, also zero position change (hold start position)
            fit_p1 = p0.copy() if decouple_vel else p1

            coeffs_axis_p = []
            coeffs_axis_th = []
            for j in range(3):
                # Scale velocity by h and acceleration by h^2 for tau-space BCs
                b_p = np.array([p0[j], fit_v0[j] * h, fit_a0[j] * h**2,
                                fit_p1[j], fit_v1[j] * h, fit_a1[j] * h**2])
                b_th = np.array([th0[j], w0[j] * h, al0[j] * h**2,
                                 th1[j], w1[j] * h, al1[j] * h**2])
                coeffs_axis_p.append(np.linalg.solve(M, b_p))
                coeffs_axis_th.append(np.linalg.solve(M, b_th))
            self.coeffs_pos.append((np.array(coeffs_axis_p), (t0, t1)))
            self.coeffs_ang.append((np.array(coeffs_axis_th), (t0, t1)))

            # Store raw values and flags for this segment
            self.segment_decouple.append({
                'decouple_acc': decouple_acc,
                'decouple_vel': decouple_vel,
                'raw_acc_start': a0.copy(),
                'raw_acc_end': a1.copy(),
                'raw_vel_start': v0.copy(),
                'raw_vel_end': v1.copy(),
            })

    def _eval_quintic(self, t, coeffs, is_position=False):
        """
        Evaluate quintic polynomial at time t.

        Coefficients are in normalized time tau = (t - t0) / h where h = t1 - t0.
        Derivatives are rescaled back to real time: vel = dp/dtau / h, acc = d2p/dtau2 / h^2.

        When is_position=True and a segment has decoupled fields, the polynomial's
        velocity/acceleration are replaced with linearly interpolated raw waypoint
        values so the controller gets the intended feedforward without the polynomial
        trying to integrate them into position.

        Args:
            t: Time to evaluate (real time)
            coeffs: List of (coeff_array, (t0, t1)) tuples
            is_position: If True, apply decouple overrides from segment_decouple

        Returns:
            Tuple of (position, velocity, acceleration) in real-time units
        """
        for i, (coeff, (t0, t1)) in enumerate(coeffs):
            if t0 <= t <= t1:
                # Update current waypoint index
                self.current_waypoint_index = i
                h = t1 - t0
                if h < 1e-10:
                    h = 1e-10
                tau = (t - t0) / h
                T = np.array([1, tau, tau**2, tau**3, tau**4, tau**5])
                dT = np.array([0, 1, 2*tau, 3*tau**2, 4*tau**3, 5*tau**4])
                ddT = np.array([0, 0, 2, 6*tau, 12*tau**2, 20*tau**3])
                pos = coeff @ T
                vel = coeff @ dT / h
                acc = coeff @ ddT / h**2

                # Apply decouple overrides for position coefficients
                if is_position and hasattr(self, 'segment_decouple') and i < len(self.segment_decouple):
                    sd = self.segment_decouple[i]
                    if sd['decouple_acc']:
                        # Linearly interpolate raw acceleration across the segment
                        acc = (1 - tau) * sd['raw_acc_start'] + tau * sd['raw_acc_end']
                    if sd['decouple_vel']:
                        vel = (1 - tau) * sd['raw_vel_start'] + tau * sd['raw_vel_end']

                return pos, vel, acc
        # Handle t outside the defined range (clamp to last segment endpoint)
        self.current_waypoint_index = len(coeffs) - 1
        coeff, (t0, t1) = coeffs[-1]
        h = t1 - t0
        if h < 1e-10:
            h = 1e-10
        tau = np.clip((t - t0) / h, 0.0, 1.0)
        T = np.array([1, tau, tau**2, tau**3, tau**4, tau**5])
        dT = np.array([0, 1, 2*tau, 3*tau**2, 4*tau**3, 5*tau**4])
        ddT = np.array([0, 0, 2, 6*tau, 12*tau**2, 20*tau**3])
        pos = coeff @ T
        vel = coeff @ dT / h
        acc = coeff @ ddT / h**2

        # Apply decouple overrides for the clamped case too
        if is_position and hasattr(self, 'segment_decouple') and len(self.segment_decouple) > 0:
            sd = self.segment_decouple[-1]
            if sd['decouple_acc']:
                acc = (1 - tau) * sd['raw_acc_start'] + tau * sd['raw_acc_end']
            if sd['decouple_vel']:
                vel = (1 - tau) * sd['raw_vel_start'] + tau * sd['raw_vel_end']

        return pos, vel, acc

    def _eval_piecewise_linear(self, t):
        """
        Evaluate piecewise linear trajectory at time t.

        Uses linear interpolation between waypoints for position and angles.
        Velocity and acceleration are taken directly from the waypoints (constant per segment).

        Args:
            t: Time to evaluate

        Returns:
            Tuple of (position, velocity, acceleration, angles, omega, alpha)
        """
        # Handle time before first waypoint
        if t <= self.waypoints[0][6]:  # Index 6 is time
            self.current_waypoint_index = 0
            p0, v0, a0, th0, w0, al0, *_ = self.waypoints[0]
            return p0.copy(), v0.copy(), a0.copy(), th0.copy(), w0.copy(), al0.copy()

        # Handle time after last waypoint
        if t >= self.waypoints[-1][6]:  # Index 6 is time
            self.current_waypoint_index = len(self.waypoints) - 1
            p_last, v_last, a_last, th_last, w_last, al_last, *_ = self.waypoints[-1]
            return p_last.copy(), v_last.copy(), a_last.copy(), th_last.copy(), w_last.copy(), al_last.copy()

        # Find the segment containing time t
        for i in range(len(self.waypoints) - 1):
            p0, v0, a0, th0, w0, al0, t0, *_ = self.waypoints[i]
            p1, v1, a1, th1, w1, al1, t1, *_ = self.waypoints[i + 1]

            if t0 <= t <= t1:
                # Update current waypoint index
                self.current_waypoint_index = i

                # Linear interpolation parameter
                if t1 - t0 > 1e-10:  # Avoid division by zero
                    alpha = (t - t0) / (t1 - t0)
                else:
                    alpha = 0.0

                # Linearly interpolate position and angles
                pos = (1 - alpha) * p0 + alpha * p1
                angles = (1 - alpha) * th0 + alpha * th1

                # For velocity and acceleration, use the values at the current waypoint
                # This creates a step function for vel/acc
                vel = v0.copy()
                acc = a0.copy()
                omega = w0.copy()
                alpha_ang = al0.copy()

                return pos, vel, acc, angles, omega, alpha_ang

        # Should never reach here, but return last waypoint as fallback
        self.current_waypoint_index = len(self.waypoints) - 1
        p_last, v_last, a_last, th_last, w_last, al_last, *_ = self.waypoints[-1]
        return p_last.copy(), v_last.copy(), a_last.copy(), th_last.copy(), w_last.copy(), al_last.copy()

    def _get_at_time(self, t):
        """
        Gets the raw trajectory state at a specific time t.

        Args:
            t: Time to evaluate

        Returns:
            Tuple of (position, velocity, acceleration, angles, omega, alpha)
        """
        if self.init_type == 'equations':
            x = self.pos_func(t)
            v = self.vel_func(t)
            a = self.acc_func(t) if self.acc_func else np.zeros(3)
            theta = self.angle_func(t)
            omega = self.omega_func(t)
            alpha = self.alpha_func(t) if self.alpha_func else np.zeros(3)
        elif self.init_type == 'quintic_polynomial':
            x, v, a = self._eval_quintic(t, self.coeffs_pos, is_position=True)
            theta, omega, alpha = self._eval_quintic(t, self.coeffs_ang)
        else:  # piecewise_linear
            x, v, a, theta, omega, alpha = self._eval_piecewise_linear(t)
        return x, v, a, theta, omega, alpha

    def get(self, t, robot_pos=None):
        """
        Obtain the desired waypoint at real time t.

        If robot_pos is provided, it dynamically adjusts the progression along
        the trajectory to help the robot keep up.

        Args:
            t (float): The current real-world time.
            robot_pos (np.array, optional): The robot's current 3D position.
                                            Defaults to None.

        Returns:
            tuple: The desired state (x, v, a, theta, omega, alpha) for the robot.
        """
        # If no robot state is given, just behave as before but using virtual time.
        if robot_pos is None:
            # We still advance virtual_t by real_t's delta to keep it moving
            dt = t - self.last_real_t
            self.virtual_t += dt
            self.last_real_t = t
            return self._get_at_time(self.virtual_t)

        # Dynamic Time Scaling Logic
        # 1. Get the desired state at the current virtual time
        p_desired, v_desired, _, _, _, _ = self._get_at_time(self.virtual_t)

        # 2. Calculate the "along-path" error
        error_vec = p_desired - robot_pos
        v_norm = np.linalg.norm(v_desired)

        e_along_path = 0.0
        if v_norm > 1e-6:  # Avoid division by zero if velocity is zero
            v_dir = v_desired / v_norm
            e_along_path = np.dot(error_vec, v_dir)

        # 3. Calculate the time scaling factor and update virtual time
        dt = t - self.last_real_t
        scale = 1.0 - self.time_scaling_gain * e_along_path

        # Prevent time from going backwards or jumping too far ahead
        scale = np.clip(scale, 0.0, 2.0)

        self.virtual_t += scale * dt
        self.last_real_t = t

        # 4. Return the waypoint at the new virtual time
        return self._get_at_time(self.virtual_t)

    def warm_start(self, dt, total_time):
        """
        Pre-compute trajectory points for visualization or analysis.

        Args:
            dt: Time step for sampling
            total_time: Total duration to compute

        Returns:
            List of trajectory points (time, position, velocity, etc.)
        """
        self.traj = []
        # Reset time state on warm_start
        self.virtual_t = 0.0
        if (self.init_type == 'quintic_polynomial' or self.init_type == 'piecewise_linear') and self.waypoints:
            self.virtual_t = self.waypoints[0][6]  # Time is at index 6, not -1 (which is l1_params)
        self.last_real_t = self.virtual_t

        # Use a temporary variable for time to not mess with the class state
        time_iter = self.virtual_t
        end_time = time_iter + total_time
        while time_iter <= end_time:
            self.traj.append((time_iter, *self._get_at_time(time_iter)))
            time_iter += dt

    def get_current_waypoint_l1_params(self):
        """
        Get L1 adaptive control parameters for the current waypoint.

        Returns:
            dict or None: L1 parameters if defined for current waypoint, None otherwise
                         {'enable': int, 'enable_force': int, 'enable_moment': int}
        """
        if self.init_type not in ['quintic_polynomial', 'piecewise_linear']:
            return None

        if not hasattr(self, 'waypoints') or not self.waypoints:
            return None

        # Make sure index is valid
        if self.current_waypoint_index < 0 or self.current_waypoint_index >= len(self.waypoints):
            return None

        # Get L1 params from current waypoint (index 7 in the tuple)
        waypoint = self.waypoints[self.current_waypoint_index]
        if len(waypoint) > 7:
            return waypoint[7]  # l1_params
        return None
