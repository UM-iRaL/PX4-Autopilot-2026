# L1 Adaptive Control Augmentation for Geometric Controller

## Overview

This module implements **L1 adaptive control augmentation** for the geometric position controller used with omnidirectional (tilting) quadcopters. L1 adaptive control provides robust performance in the presence of model uncertainties and external disturbances by estimating and compensating for unknown dynamics in real-time.

### Integration Point

The L1 augmentation is applied **after** the geometric controller computes nominal force and moment commands, but **before** control allocation distributes them to individual motors and servos:

```
Geometric Controller → L1 Augmentation → Control Allocation → Motors/Servos
   (nominal F, M)    → (adaptive ΔF, ΔM) → (α, β, thrust)  → (PWM commands)
```

---

## File Structure

```
L1AdaptiveController/
├── README.md                      # This file
├── l1_adaptive_controller.hpp     # Class definition and interface
├── l1_adaptive_controller.cpp     # Core L1 algorithm implementation
├── l1_adaptive_params.c           # PX4 parameter definitions
└── CMakeLists.txt                 # Build configuration
```

### Key Components

- **State Predictor**: Estimates vehicle velocity and angular velocity using model + adaptive terms
- **Uncertainty Estimator**: Computes matched uncertainties from prediction errors
- **Low-Pass Filters**: Filters adaptive commands for stability (1 for force, 2 cascaded for moments)
- **Parameter Manager**: Runtime-configurable adaptation and filter parameters

---

## Algorithm Description

### State Prediction

The L1 controller maintains a state predictor that estimates the vehicle's velocity and angular velocity:

**Velocity Predictor (NED frame):**
```
v̂ = v̂_prev + (g·e₃ - R·(F_nom + F_ad + σ_F)/m + ε_v·As_v)·dt
```

**Angular Velocity Predictor (body frame):**
```
ω̂ = ω̂_prev + (J⁻¹·(M_nom + M_ad + σ_M - ω×Jω) + ε_ω·As_ω)·dt
```

Where:
- `v̂, ω̂` = Predicted velocity and angular velocity
- `F_nom, M_nom` = Nominal force and moment from geometric controller
- `F_ad, M_ad` = Adaptive force and moment corrections
- `σ_F, σ_M` = Estimated uncertainties
- `As_v, As_ω` = State predictor poles (negative values)
- `ε_v, ε_ω` = Prediction errors (v̂ - v, ω̂ - ω)

### Uncertainty Estimation

From prediction errors, the controller estimates matched uncertainties:

**Force Uncertainty (body frame):**
```
σ_F = m · R^T · Φ⁻¹(As_v, dt) · μ_v
```

**Moment Uncertainty (body frame):**
```
σ_M = -J · Φ⁻¹(As_ω, dt) · μ_ω
```

Where `Φ⁻¹(As, dt)` is the inverse of the state transition matrix, and `μ_v, μ_ω` are the prediction errors.

### Low-Pass Filtering

To ensure stability and smooth control, adaptive terms are filtered:

**Force Channel (single filter):**
```
F_ad = -LPF₁(σ_F, ωc_force)
```

**Moment Channel (cascaded filters for better phase margin):**
```
M_ad = -LPF₂(LPF₁(σ_M, ωc_moment1), ωc_moment2)
```

The negative sign compensates for the estimated uncertainties.

---

## Parameters

All L1 parameters are in the **"L1 Adaptive Control"** parameter group.

| Parameter | Type | Default | Range | Description |
|-----------|------|---------|-------|-------------|
| **L1AD_ACTIVE** | INT32 | 0 | [0, 1] | Enable (1) or disable (0) L1 augmentation |
| **L1AD_VEL_AS** | FLOAT | -5.0 | [-20.0, -0.5] | Velocity state predictor pole (1/s). More negative = faster adaptation |
| **L1AD_ANG_AS** | FLOAT | -5.0 | [-20.0, -0.5] | Angular velocity state predictor pole (1/s). More negative = faster adaptation |
| **L1AD_FRC_FREQ** | FLOAT | 50.0 | [5.0, 200.0] | Force channel low-pass filter cutoff frequency (Hz) |
| **L1AD_MOM_FQ1** | FLOAT | 50.0 | [5.0, 200.0] | Moment channel first filter cutoff frequency (Hz) |
| **L1AD_MOM_FQ2** | FLOAT | 30.0 | [5.0, 100.0] | Moment channel second filter cutoff frequency (Hz). Should be ≤ L1AD_MOM_FQ1 |

## Usage

### Enabling L1 Adaptive Control

#### Option 1: QGroundControl (Recommended for Testing)

1. Connect to your vehicle (SITL or hardware)
2. Navigate to **Vehicle Setup → Parameters**
3. Search for **"L1 Adaptive Control"** group
4. Set `L1AD_ACTIVE` = **1**
5. Adjust other parameters as needed
6. Click **"Save"** and reboot if necessary

#### Option 2: MAVLink Console

```bash
# In PX4 console (pxh>)
param set L1AD_ACTIVE 1

# Optional: customize parameters
param set L1AD_VEL_AS -10.0
param set L1AD_ANG_AS -10.0
param set L1AD_FRC_FREQ 5.0
param set L1AD_MOM_FQ1 1.5
param set L1AD_MOM_FQ2 0.5
param save
```