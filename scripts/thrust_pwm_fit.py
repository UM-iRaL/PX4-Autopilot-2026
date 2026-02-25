import argparse, numpy as np, pandas as pd, matplotlib.pyplot as plt

def alpha_given_omegamax(omega_max, u, rpm, s_min=0.15):
    s = rpm / omega_max
    # Exclude low throttle region where ESC/motor dynamics are non-linear
    mask = (s > s_min) & (s < 0.98)
    if mask.sum() < 5: return np.nan, np.inf
    s, u2 = s[mask], u[mask]
    x = s**2 - s
    y = u2 - s
    denom = float(x @ x)
    if not np.isfinite(denom) or denom <= 0: return np.nan, np.inf
    alpha = float((y @ x) / denom)
    alpha = float(np.clip(alpha, 0.0, 1.0))
    u_fit = s + alpha * (s**2 - s)
    sse = float(np.sum((u2 - u_fit)**2))
    return alpha, sse

def main(csv_path, num_motors=2):
    df = pd.read_csv(csv_path, encoding='utf-8-sig')
    # Remove trailing comma columns and any unnamed columns
    df = df.drop(columns=[c for c in df.columns if c.startswith("Unnamed:") or str(c).strip() == ""], errors="ignore")

    # Coerce to numeric (bad encodings -> NaN)
    for col in ["ESC signal (µs)", "Motor Optical Speed (RPM)", "Motor Electrical Speed (RPM)", "Thrust (N)", "Thrust (kgf)"]:
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")

    # Pick RPM column with most finite nonzero data
    rpm_candidates = [c for c in ["Motor Optical Speed (RPM)", "Motor Electrical Speed (RPM)"] if c in df.columns]
    if not rpm_candidates:
        raise RuntimeError(f"No RPM columns found. Have: {df.columns.tolist()}")

    stats = []
    for c in rpm_candidates:
        v = df[c].to_numpy(float)
        finite = np.isfinite(v)
        nz = np.sum(np.abs(v[finite]) > 1.0)  # >1 RPM
        stats.append((nz, c))
    stats.sort(reverse=True)
    rpm_col = stats[0][1]

    if "ESC signal (µs)" not in df.columns:
        raise RuntimeError("ESC signal (µs) column missing.")

    esc = df["ESC signal (µs)"].to_numpy(float)
    rpm = df[rpm_col].to_numpy(float)

    # Basic finite mask first
    m0 = np.isfinite(esc) & np.isfinite(rpm)
    esc, rpm = esc[m0], rpm[m0]

    if esc.size == 0 or rpm.size == 0:
        raise RuntimeError("No valid data after basic finite filtering.")

    # Remove obvious idle / no-rotation rows using percentiles
    # Keep rows above a small RPM floor derived from data (5th percentile of nonzero RPM)
    rpm_pos = np.abs(rpm[rpm > 0])
    if rpm_pos.size == 0:
        raise RuntimeError("RPM appears to be zero everywhere. Check sensor column.")
    rpm_floor = np.percentile(rpm_pos, 1.0)
    m = (rpm > rpm_floor)
    esc, rpm = esc[m], rpm[m]

    # Normalize ESC µs -> u in [0,1] using robust min/max
    us_min = np.percentile(esc, 1); us_max = np.percentile(esc, 99)
    if not np.isfinite(us_min) or not np.isfinite(us_max) or (us_max - us_min) < 200:
        # fallback to common 1000–2000 µs range
        us_min, us_max = 1000.0, 2000.0
    u = np.clip((esc - us_min) / (us_max - us_min), 0.0, 1.0)

    # Also drop the very bottom throttle tail where RPM may still be unreliable
    u_floor = np.percentile(u, 1.0)
    keep = u >= u_floor
    u, rpm = u[keep], rpm[keep]

    if u.size < 10:
        raise RuntimeError("Too few samples after trimming. Try a longer sweep.")

    # Calculate thrust coefficient if thrust data is available (before binning)
    ct = None
    thrust_col = None
    if "Thrust (N)" in df.columns:
        thrust_col = "Thrust (N)"
    elif "Thrust (kgf)" in df.columns:
        thrust_col = "Thrust (kgf)"

    thrust_data_for_bin = None
    if thrust_col is not None:
        thrust_data = df[thrust_col].to_numpy(float)
        # Convert kgf to N if needed (1 kgf = 9.80665 N)
        if thrust_col == "Thrust (kgf)":
            thrust_data = thrust_data * 9.80665
        # Use original indices to match thrust with rpm
        thrust_data_for_bin = thrust_data[m0][m][keep]

    # Bin throttle to reduce noise (0.1% bins)
    ub = np.round(u, 3)
    if thrust_data_for_bin is not None:
        tmp = pd.DataFrame({"u": ub, "rpm": rpm, "thrust": thrust_data_for_bin}).groupby("u", as_index=False).median()
        u = tmp["u"].to_numpy()
        rpm = tmp["rpm"].to_numpy()
        thrust = tmp["thrust"].to_numpy()
    else:
        tmp = pd.DataFrame({"u": ub, "rpm": rpm}).groupby("u", as_index=False).median()
        u = tmp["u"].to_numpy()
        rpm = tmp["rpm"].to_numpy()

    # Sweep omega_max near the upper RPM envelope
    omega_p95 = np.percentile(rpm, 99)
    omega_max_guess = omega_p95 if np.isfinite(omega_p95) and omega_p95 > 0 else np.max(rpm)
    grid = np.linspace(0.85*omega_max_guess, 1.20*omega_max_guess, 251)

    best = {"omega_max": None, "alpha": None, "sse": np.inf}
    for om in grid:
        a, e = alpha_given_omegamax(om, u, rpm)
        if np.isfinite(e) and e < best["sse"]:
            best.update({"omega_max": om, "alpha": a, "sse": e})

    if best["omega_max"] is None or not np.isfinite(best["alpha"]):
        # Diagnostics
        print("Diagnostics:")
        print(f"- rpm column used: {rpm_col}")
        print(f"- ESC µs min/max: {esc.min():.1f} / {esc.max():.1f}")
        print(f"- RPM min/max: {rpm.min():.1f} / {rpm.max():.1f}")
        print(f"- Samples after trim: {rpm.size}")
        raise RuntimeError("Fit failed: data likely stuck near zero or saturated.")

    alpha = best["alpha"]
    omega_max_rpm = best["omega_max"]
    omega_max_rads = omega_max_rpm * 2*np.pi/60.0

    # Calculate thrust coefficient if thrust data is available
    if thrust_col is not None:
        # Take absolute value of thrust (in case sensor is mounted upside down)
        thrust = np.abs(thrust)

        # Convert RPM to rad/s
        omega_rads = rpm * 2*np.pi/60.0

        # Fit: Thrust = ct * omega^2
        # Use least squares: ct = (thrust @ omega^2) / (omega^2 @ omega^2)
        mask_ct = (omega_rads > 0) & (thrust > 0) & np.isfinite(thrust)
        if mask_ct.sum() > 5:
            omega_sq = omega_rads[mask_ct]**2
            thrust_fit = thrust[mask_ct]
            ct = float((thrust_fit @ omega_sq) / (omega_sq @ omega_sq))

    # Report
    print(f"Using RPM column: {rpm_col}")
    print(f"ESC µs range (robust): [{us_min:.1f}, {us_max:.1f}]")
    print(f"RPM range (kept): [{rpm.min():.0f}, {rpm.max():.0f}]   n={rpm.size}")
    print(f"Number of motors in test: {num_motors}")
    print(f"Estimated alpha     = {alpha:.3f}")
    print(f"Estimated omega_max = {omega_max_rpm:.1f} RPM ({omega_max_rads:.1f} rad/s)")
    if ct is not None:
        ct_total = ct
        ct_per_motor = ct / num_motors
        print(f"\nThrust coefficient (total): {ct_total:.6e} N·s²/rad²")
        print(f"Thrust coefficient (per motor): {ct_per_motor:.6e} N·s²/rad²")
    print("\n// PX4 constants:")
    print(f"constexpr float kOmegaMax = {omega_max_rads:.1f}f;  // rad/s")
    print(f"constexpr float kAlpha    = {alpha:.3f}f;")
    if ct is not None:
        print(f"constexpr float kCt_total      = {ct_total:.6e}f;  // N·s²/rad² (total for {num_motors} motors)")
        print(f"constexpr float kCt_per_motor  = {ct_per_motor:.6e}f;  // N·s²/rad² (single motor)")

    # Plot
    if ct is not None:
        fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))
        fig.suptitle(f"Motor Thrust Analysis - {num_motors} Coaxial Motor{'s' if num_motors != 1 else ''}", fontsize=14, fontweight='bold')
    else:
        fig, ax1 = plt.subplots(1, 1, figsize=(6, 5))
        fig.suptitle(f"Motor PWM Analysis - {num_motors} Coaxial Motor{'s' if num_motors != 1 else ''}", fontsize=14, fontweight='bold')

    # Plot 1: PWM model
    s = rpm / omega_max_rpm
    u_fit = s + alpha*(s**2 - s)
    mask = (s >= 0) & (s <= 1)
    s_plot, u_plot, u_fit_plot = s[mask], u[mask], u_fit[mask]

    # Show which region was used for fitting
    s_min_fit = 0.15
    fit_region_mask = (s_plot >= s_min_fit) & (s_plot <= 0.98)

    # Linear fit for comparison: u = a * s + b
    s_linear = s_plot[fit_region_mask]
    u_linear = u_plot[fit_region_mask]
    A = np.vstack([s_linear, np.ones(len(s_linear))]).T
    a_lin, b_lin = np.linalg.lstsq(A, u_linear, rcond=None)[0]
    u_linear_fit = a_lin * s_plot + b_lin

    order = np.argsort(s_plot)
    ax1.scatter(s_plot[~fit_region_mask], u_plot[~fit_region_mask], s=12, alpha=0.3, color='gray', label="Excluded from fit")
    ax1.scatter(s_plot[fit_region_mask], u_plot[fit_region_mask], s=12, label="Used for fit")
    ax1.plot(s_plot[order], u_fit_plot[order], linewidth=2, color='C1', label=f"Quadratic (α={alpha:.3f})")
    ax1.plot(s_plot[order], u_linear_fit[order], linewidth=2, linestyle='--', color='C2', label=f"Linear (u={a_lin:.3f}s+{b_lin:.3f})")
    ax1.axvline(s_min_fit, color='red', linestyle='--', alpha=0.5, linewidth=1)
    ax1.set_xlabel("s = ω / ω_max")
    ax1.set_ylabel("u (normalized throttle)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)

    # Plot 2: Thrust coefficient
    if ct is not None:
        omega_rads_all = rpm * 2*np.pi/60.0
        mask_ct = (omega_rads_all > 0) & (thrust > 0) & np.isfinite(thrust)
        omega_sq = omega_rads_all[mask_ct]**2
        thrust_measured = thrust[mask_ct]
        thrust_model = ct * omega_sq
        order2 = np.argsort(omega_sq)
        ax2.scatter(omega_sq, thrust_measured, s=12, label="Measured")
        ax2.plot(omega_sq[order2], thrust_model[order2], linewidth=2, label=f"T = {ct:.3e}·ω²")
        ax2.set_xlabel("ω² (rad²/s²)")
        ax2.set_ylabel("Thrust (N)")
        ax2.legend()
        ax2.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.show()

if __name__ == "__main__":
    ap = argparse.ArgumentParser()
    ap.add_argument("csv", nargs="?", default="scripts/rcbenchmark_raw.csv", help="Path to RCbenchmark CSV")
    ap.add_argument("--num-motors", type=int, default=2, help="Number of coaxial motors in test setup (default: 2)")
    args = ap.parse_args()
    main(args.csv, args.num_motors)