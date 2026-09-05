# Balance PID — Reference

**Project:** OMNIS · cascaded balance control, no encoders
**Scope:** Discrete forms, units, anti-windup, the outer loop, tuning procedure.
**Implements:** `omnis-info.md` §13

---

## 1. The honest starting point

§13a states the ceiling plainly and it is repeated here because every design
choice below follows from it:

> Best-performing self-balancing robots close an inner angle loop **and** an
> outer wheel-velocity loop using encoder feedback. This design has no wheel
> encoders — the A4988s are open-loop — so true closed-loop velocity control is
> not available with the current BOM.

What follows is the best achievable architecture *without* encoders. It works —
it is what most hobbyist stepper balancers run — but it has a real ceiling that
encoders would remove. **A wheel encoder pair is the single highest-leverage
hardware addition to this project.**

---

## 2. Architecture

```
   RC sticks                            ┌──────────────────┐
       │                                │  trim (§7d pot)  │
       ▼                                └────────┬─────────┘
  fore/aft bias ──┐                              │
                  ▼                              ▼
        ┌─────────────────────┐        ┌───────────────────┐
        │ OUTER  ~20 Hz       │        │                   │
        │ vel_bias_t          │───────▶│  target lean      │
        │ integral of         │        │                   │
        │ commanded step rate │        └─────────┬─────────┘
        └─────────▲───────────┘                  │
                  │                              ▼
                  │                    ┌───────────────────┐
                  │   fused pitch ────▶│ INNER  500 Hz     │
                  │   gyro rate   ────▶│ pid_t             │
                  │                    └─────────┬─────────┘
                  │                              │ commanded accel
                  │                              ▼
                  │                    ┌───────────────────┐
                  └────────────────────│ integrate -> step │
                    commanded rate     │ rate, accel-limit │
                                       └─────────┬─────────┘
                                                 ▼
                                          RMT step generation
```

| Loop | Rate | Input | Output |
|---|---|---|---|
| Inner | 500 Hz (§11d) | target lean, fused pitch, raw gyro | wheel **acceleration** |
| Outer | ~20 Hz | commanded step rate | **target lean** |

Turn is a differential step-rate bias added *after* the fore-aft PID output.

---

## 3. Inner loop — discrete form

```
error = setpoint - measurement
P     = kp * error
I     = clamp( I + ki*error*dt ),  frozen if it deepens saturation
D     = -kd * measurement_rate
out   = clamp( P + I + D )
```

### Two deliberate departures from textbook PID

**1. Derivative on measurement, from the gyro directly.**

§13b: *"angle_rate taken directly from gyro rather than differentiated from the
EKF output, for lower latency"*. So `pid_update()` takes the rate as an
argument rather than differencing the error.

Since `d(error)/dt = d(setpoint)/dt − d(measurement)/dt` and the setpoint moves
slowly relative to a 500 Hz loop, the first term drops and `D = −kd · rate`.
**The minus sign is that dropped derivation, not a sign error.**

Two benefits beyond latency: no derivative kick when the setpoint steps, and no
amplification of EKF quantisation noise by a `1/dt` = 500 factor.

**2. Conditional-integration anti-windup.**

On an open-loop stepper balancer the output saturates often (§13c). Plain
integral clamping still lets the integrator wind all the way to its clamp while
the actuator is pinned, and that stored energy comes back out as an overshoot at
exactly the moment the robot is trying to recover.

Instead the integrator is **frozen** whenever the new value would push further
into saturation, and released the instant the error reverses:

```
1. compute candidate integral, do not commit
2. form tentative output
3. if saturated AND candidate moves further into saturation -> discard
4. recompute output from the committed integral, clamp
```

### Units

The PID itself is unit-agnostic; the gains carry the units.

| Quantity | Unit |
|---|---|
| `error`, `setpoint`, `measurement` | rad |
| `measurement_rate` | rad/s |
| output | microsteps/s² (commanded wheel acceleration) |
| `kp` | (µsteps/s²) / rad |
| `ki` | (µsteps/s²) / (rad·s) |
| `kd` | (µsteps/s²) / (rad/s) |

**Physical meaning of `kp`:** how hard the wheels accelerate per radian of lean
error. If 1° of lean should produce ~500 µsteps/s² of correction, then
`kp ≈ 500 / 0.01745 ≈ 28600`. Work in the units you will actually pass; the
numbers are large and that is fine.

---

## 4. Outer loop — velocity bias

With no encoders there is no velocity feedback, so §13b uses the **integral of
commanded step rate** as a proxy for sustained drive effort:

```
integral += commanded_step_rate * dt
target_lean = clamp( -gain * integral + trim, ±lean_limit )
```

**The negative sign is critical.** Forward drive effort commands a *backward*
lean, because leaning back is how a balancing robot decelerates. Getting this
sign wrong produces a robot that accelerates until it falls over, and it is the
most common bug in this loop.

Integrating steps/s over seconds gives **steps** — a displacement proxy, not a
velocity one. That is deliberate: nulling accumulated displacement effort makes
the robot return toward where it started rather than merely stopping wherever it
drifted to.

`lean_limit` defaults to **6°**. The outer loop must never be able to command a
lean the inner loop cannot recover from; 6° is well inside the recoverable
envelope for a platform of this size and can be raised once the inner loop is
trusted.

`trim` is where §7d's IMU trim pot lands — a static offset on the *outer loop's
target*, compensating an off-centre CG. §13b is explicit that this is the
correct place for it, **not** a bias on the raw EKF angle.

### Verified — `gain = 1.2e-5`, `lean_limit = 6°`, 20 Hz, sustained 3000 steps/s

| t (s) | integral | target lean (°) |
|---|---|---|
| 0.05 | 150 | −0.1031 |
| 0.25 | 750 | −0.5157 |
| 0.50 | 1500 | −1.0313 |
| 1.00 | 3000 | −2.0626 |
| 2.50 | 7500 | −5.1566 |
| 5.00 | 15000 | −6.0000 (clamped) |

With `trim = 1.5°` and zero drive: target = **+1.5000°**.

### The weak link

The proxy believes every commanded step actually happened. A slipped or lost
step is **invisible** here and shows up as slow drift. §13c's mitigations are
preventive, not corrective: generous A4988 current margin (§3f `Vref`) and
acceleration limits in the step-rate path so the outer loop never asks for more
than the motor can deliver.

---

## 5. Tuning procedure

**No numeric gains are given.** §13d declines to guess them without mass, CG
height and wheel radius, and that refusal is correct — a number offered without
those is a guess dressed up as a spec. `pid_config_defaults()` therefore sets
all gains to **zero**: a zero-gain controller does nothing, which is visible and
safe, whereas a guessed-gain controller looks like it works and falls over.

Tune in this order, on a tether rig or soft surface:

1. **Verify the estimator first.** Before any gain is non-zero, confirm the
   fused pitch tracks real tilt with the right sign and no visible lag. Tuning a
   PID against a broken estimate is unbounded frustration.
2. **`kp` alone.** `ki = kd = 0`. Raise until the robot oscillates steadily
   around upright, then **back off ~30%**.
3. **Add `kd`** to damp the residual oscillation. Too much makes it buzz and
   amplifies gyro noise into the motors — audible before it is visible.
4. **Add `ki` last, small.** Only to correct steady-state lean. Too aggressive
   and it fights the outer velocity-bias loop.
5. **Only then enable the outer loop** (`vel_bias.gain` from 0). Raise until
   the robot stops drifting across the room without visibly hunting.

### Sanity checks while tuning

| Symptom | Likely cause |
|---|---|
| Accelerates away and falls | outer-loop sign wrong (§4) |
| Falls the instant it is released | `kp` sign wrong, or mounting not corrected |
| Slow oscillation, ~1 s period | `kp` too high, or `ki` fighting the outer loop |
| Audible buzz, no visible motion | `kd` too high |
| Balances then drifts steadily | outer-loop gain too low, or lost steps |
| Recovers, then overshoots hard | integrator windup — check anti-windup is active |

---

## 6. Verified numbers

`./run_host_tests.sh` — 67 assertions, 0 failures.

### PID arithmetic — `kp=8, ki=2, kd=0.4`, out ±100, `imax=30`, `dt=0.002`

| n | setpoint | meas | rate | P | I | D | out |
|---|---|---|---|---|---|---|---|
| 1 | 0.00 | 0.000 | 0.00 | 0.00000 | 0.000000 | 0.00000 | **0.00000** |
| 2 | 0.00 | −0.050 | 0.00 | 0.40000 | 0.000200 | 0.00000 | **0.40020** |
| 3 | 0.00 | −0.050 | −0.10 | 0.40000 | 0.000400 | 0.04000 | **0.44040** |
| 4 | 0.00 | −0.050 | −0.10 | 0.40000 | 0.000600 | 0.04000 | **0.44060** |
| 5 | 0.00 | −0.050 | −0.10 | 0.40000 | 0.000800 | 0.04000 | **0.44080** |
| 6 | 0.00 | 0.000 | 0.20 | 0.00000 | 0.000800 | −0.08000 | **−0.07920** |
| 7 | 0.00 | 0.100 | 0.20 | −0.80000 | 0.000400 | −0.08000 | **−0.87960** |

### Anti-windup — error held at −20 rad, output pinned at the −100 floor

| t (s) | P | I | out |
|---|---|---|---|
| 0.00 | −160.00 | **0.000000** | −100.00 |
| 0.20 | −160.00 | **0.000000** | −100.00 |
| 1.00 | −160.00 | **0.000000** | −100.00 |
| 6.00 | −160.00 | **0.000000** | −100.00 |

Integrator frozen at exactly zero through six seconds of saturation. On the
first tick after the error reverses it is free again: `I = 0.004000`,
`out = 8.004`.

A plain clamped integrator would have wound to its ±30 cap within 15 seconds and
then had to unwind through 30 units of spurious output during recovery.
