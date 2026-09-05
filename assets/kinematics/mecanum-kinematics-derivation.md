# Mecanum Kinematics — Full Symbolic Derivation

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Scope:** Complete first-principles derivation of the inverse and forward
kinematics for the OMNIS drivetrain, kept fully symbolic in
`wheel_radius_mm`, `wheelbase_mm`, and `track_width_mm`.

> This file is the *why*. For the equations alone (no derivation), see
> [`mecanum-kinematics-reference.md`](mecanum-kinematics-reference.md).
> For the firmware, see [`mecanum_kinematics.c`](mecanum_kinematics.c) and
> [`mecanum-kinematics-code-explained.md`](mecanum-kinematics-code-explained.md).

---

## 1. Hardware assumptions

These are the physical facts the derivation rests on. If any of them change,
the affected section is flagged inline.

| Property | Value | Affects |
|---|---|---|
| Wheel type | 60 mm mecanum, rollers at **45°** to the wheel plane | §4 — the 45° is what makes the mixing coefficients clean ±1 |
| Layout | 4 wheels, rectangular, all axles parallel to the body Y axis | §3, §5 |
| Roller handedness | FL + RR share one tilt; FR + RL share the mirrored tilt (standard X-drive) | §5 |
| Motor | 2-phase NEMA17-class, 200 full steps/rev (1.8°/step) | §7 |
| Driver | A4988, **fixed** 1/16 microstepping (MS1/MS2/MS3 tied HIGH) | §7 |
| Resulting resolution | 200 × 16 = **3200 microsteps/rev** | §7 |
| Feedback | **None** — open-loop steppers, no encoders | §9 |

The microstepping figure is a hard-wired constant, not a runtime parameter.
The three geometry values are runtime parameters read from `params.json` and
must never be baked into the equations.

---

## 2. Coordinate and sign conventions

Right-handed body frame, standard robotics convention (ROS REP-103):

| Symbol | Quantity | Positive direction | Unit |
|---|---|---|---|
| `vx` | body longitudinal velocity | **forward** | mm/s |
| `vy` | body lateral velocity (strafe) | **left** | mm/s |
| `w` (ω) | body yaw rate | **counter-clockwise**, viewed from above | rad/s |
| `ω_i` | angular velocity of wheel *i* | the sense that drives the robot **forward** | rad/s |
| `f_i` | step-pulse rate for wheel *i* | sign follows `ω_i` | microsteps/s |

### Unit discipline

`wheel_radius_mm`, `wheelbase_mm`, and `track_width_mm` are in **millimetres**,
so `vx` and `vy` must also be in **mm/s**. Yaw rate `w` is in **rad/s**, which
makes the product `(length in mm) × (rad/s)` come out in mm/s — dimensionally
consistent with `vx` and `vy`.

Feeding m/s into these equations produces a yaw contribution that is wrong by a
factor of 1000 while forward motion still looks correct — a failure mode that
only shows up once you command rotation. Normalise units at the RC input layer,
once, and never again.

### Wheel angular velocity is a *mathematical* sign

`ω_i > 0` means "this wheel is turning in the sense that pushes the robot
forward". It is **not** a statement about phase order on the motor. The left-side
motors are physically mirrored relative to the right-side motors, so realising
`ω_i > 0` on FL and FR generally requires opposite `DIR` pin levels. That
mirroring belongs in the GPIO layer as a per-wheel invert flag — see §10.

---

## 3. Wheel positions in the body frame

Origin at the geometric centre of the four wheel contact patches.
`wheelbase_mm` is the front-to-back wheel-centre distance; `track_width_mm` is
the left-to-right wheel-centre distance.

```
                    +x (forward)
                        ^
                        |
        FL  o───────────┼───────────o  FR
            │           |           │
            │           |           │      wheelbase_mm
  +y <──────┼───────────O           │      = front-to-back
  (left)    │                       │
            │                       │
        RL  o───────────────────────o  RR
            <────── track_width_mm ─────>
                    (left-to-right)
```

So:

```
x_FL = +wheelbase_mm/2      y_FL = +track_width_mm/2
x_FR = +wheelbase_mm/2      y_FR = -track_width_mm/2
x_RL = -wheelbase_mm/2      y_RL = +track_width_mm/2
x_RR = -wheelbase_mm/2      y_RR = -track_width_mm/2
```

Compactly: `x_i = ±wheelbase_mm/2` (front positive),
`y_i = ±track_width_mm/2` (left positive).

---

## 4. Step 1 — velocity of each wheel's contact patch

For a rigid body moving in a plane, the velocity of a point at body-frame
position `r_i = (x_i, y_i)` is the body translation plus the rotational
contribution:

```
v_i = v_body + (w * ẑ) × r_i
```

Expanding the cross product with `ẑ = (0, 0, 1)`:

```
(w * ẑ) × (x_i, y_i, 0) = w * (-y_i, x_i, 0)
```

Therefore the contact patch of wheel *i* moves at:

```
v_i = ( vx - w*y_i ,  vy + w*x_i )                              ... (4.1)
```

Sanity check on the signs: a wheel on the **left** (`y_i > 0`) under
counter-clockwise rotation (`w > 0`) gets a **negative** x-component — it moves
backward. That is correct for a left turn.

Note (4.1) contains no wheel-specific physics yet. It is pure rigid-body
mechanics and would be identical for any wheel type.

---

## 5. Step 2 — the mecanum roller constraint

This is where the wheel type enters.

A conventional wheel imposes two constraints: it rolls along its heading and
cannot slide sideways. A mecanum wheel relaxes the second one. Its rim carries
free-spinning rollers mounted at 45°, so the contact patch **can** slide freely
in one direction — the direction perpendicular to the roller's own axis.

The consequence: the wheel only transmits force along the **roller axis
direction**. Motion perpendicular to that axis is unconstrained, absorbed by the
roller spinning on its own bearing.

### 5.1 The constraint equation

Let `â_i` be the unit vector along wheel *i*'s roller axis, projected into the
ground plane. With rollers at 45° to the wheel plane, and the wheel heading
along the body `x̂` axis:

```
â_i = (1/√2) * (1, δ_i)          where  δ_i = ±1                ... (5.1)
```

`δ_i` is the **roller handedness** of that wheel — the sign that distinguishes
the two mirrored roller tilts. `δ = +1` puts the roller axis along the
forward-left diagonal; `δ = -1` puts it along the forward-right diagonal.

The no-slip condition applies only along `â_i`: the component of the contact
velocity in that direction must equal the component the wheel's own rotation
produces in that direction. The wheel rim drives along `x̂` at speed
`wheel_radius_mm * ω_i`, so:

```
v_i · â_i  =  (wheel_radius_mm * ω_i) * (x̂ · â_i)               ... (5.2)
```

### 5.2 Why 45° gives clean ±1 coefficients

Substituting (4.1) and (5.1) into (5.2):

```
(1/√2) * [ (vx - w*y_i) + δ_i * (vy + w*x_i) ]
        =  wheel_radius_mm * ω_i * (1/√2)
```

The `1/√2` appears on **both** sides and cancels exactly. This cancellation is
the entire reason mecanum mixing matrices contain only `+1` and `-1` and no
trigonometric constants — it is a property of the 45° roller angle specifically.
A wheel with rollers at any other angle would leave a `tan` factor on the `vy`
and `w` terms.

Cancelling and rearranging:

```
wheel_radius_mm * ω_i  =  vx  +  δ_i*vy  +  w*(δ_i*x_i - y_i)   ... (5.3)
```

Equation (5.3) is the general per-wheel result. Everything below is
substitution.

### 5.3 Assigning handedness to wheels

The confirmed physical pairing is: **FL and RR share one roller tilt; FR and RL
share the mirrored tilt.** That fixes which wheels share a `δ`, but not which
pair gets `+1`. Two candidate assignments remain, and they are mirror images of
each other.

The tie is broken by the pure-strafe motion pattern. Setting `vx = 0` and
`w = 0` in (5.3) leaves `wheel_radius_mm * ω_i = δ_i * vy` — so the sign of each
wheel's rotation under pure strafe *is* its handedness. The Wikipedia "Mecanum
wheel" article describes sideways motion as:

> "Running (all at the same speed) the diagonal wheels in one direction while
> the other diagonal in the opposite direction will result in a sideways
> movement."

Diagonal wheels oppose each other — confirming the FL/RR vs FR/RL split, and
matching the stated hardware. For strafe **left** (`vy > 0`) the conventional
X-drive result drives FR and RL forward. Therefore:

> ### Sign assignment
> **FL and RR take `δ = -1`** — roller axis `â = (1, -1)/√2`, along the
> forward-right diagonal. These take the **negative** sign on `vy`.
>
> **FR and RL take `δ = +1`** — roller axis `â = (1, +1)/√2`, along the
> forward-left diagonal. These take the **positive** sign on `vy`.

If the physical wheels turn out to be mirrored from this, swap the two `δ`
values. Every sign in the `vy` **and** `w` columns flips together — they are not
independent, because both derive from the same `δ_i` in (5.3). Do not flip one
column by hand.

### 5.4 The yaw coefficient collapses to a single lever arm

The yaw term in (5.3) is `δ_i*x_i - y_i`. Substituting positions from §3 and
handedness from §5.3, with `L = wheelbase_mm` and `W = track_width_mm`:

```
FL:  δ=-1, x=+L/2, y=+W/2   ->  (-1)(+L/2) - (+W/2)  =  -(L + W)/2
FR:  δ=+1, x=+L/2, y=-W/2   ->  (+1)(+L/2) - (-W/2)  =  +(L + W)/2
RL:  δ=+1, x=-L/2, y=+W/2   ->  (+1)(-L/2) - (+W/2)  =  -(L + W)/2
RR:  δ=-1, x=-L/2, y=-W/2   ->  (-1)(-L/2) - (-W/2)  =  +(L + W)/2
```

Every wheel produces the same magnitude, differing only in sign. Define the
**yaw lever arm**:

```
k = (wheelbase_mm + track_width_mm) / 2                          ... (5.4)
```

This is the single most important derived quantity in the drivetrain. Note it
depends on the **sum** of the two frame dimensions, not on either alone and not
on their ratio. A consequence worth internalising: for mecanum rotation, making
the frame longer and narrower by the same amount changes nothing. The two
dimensions are not separable in yaw.

---

## 6. Inverse kinematics — the wheel-speed mixing matrix

Substituting (5.4) and the handedness assignment into (5.3), and dividing
through by `wheel_radius_mm`:

```
                  ┌                       ┐
 ┌      ┐         │   1     -1     -k     │   ┌      ┐
 │ ω_FL │         │                       │   │  vx  │
 │ ω_FR │    1    │   1     +1     +k     │   │  vy  │
 │ ω_RL │  = ───  │                       │ · │   w  │
 │ ω_RR │    r    │   1     +1     -k     │   └      ┘
 └      ┘         │                       │
                  │   1     -1     +k     │
                  └                       ┘

        r = wheel_radius_mm
        k = (wheelbase_mm + track_width_mm) / 2
```

Written out:

```
ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
ω_RL = ( vx + vy - k*w ) / wheel_radius_mm
ω_RR = ( vx - vy + k*w ) / wheel_radius_mm
```

The matrix has three columns and four rows: three degrees of freedom driving
four actuators. It is over-determined, which is the formal reason a mecanum
platform can fight itself (§9.2).

### 6.1 Verification against the Wikipedia diagram set

Each row below is obtained by setting one input non-zero and reading off the
matrix column.

| Motion | Input | FL | FR | RL | RR | Wikipedia description | ✓ |
|---|---|---|---|---|---|---|---|
| Forward | `vx > 0` | + | + | + | + | "all four wheels in the same direction … longitudinal force vectors add up but the transverse vectors cancel" | ✓ |
| Strafe **left** | `vy > 0` | **−** | **+** | **+** | **−** | "the diagonal wheels in one direction while the other diagonal in the opposite direction" | ✓ |
| Rotate **CCW** | `w > 0` | **−** | **+** | **−** | **+** | "both wheels on one side in one direction while the other side in the opposite direction" | ✓ |
| Diagonal fwd-left | `vx = vy > 0` | 0 | + | + | 0 | one diagonal idles, the other drives | ✓ |

The strafe row splits along **diagonals**; the rotation row splits along
**sides**. Those two patterns are distinct, and together they fix the matrix
uniquely — no other sign assignment reproduces both.

The diagonal case is a useful bench test: commanding `vx = vy` should leave FL
and RR completely stationary. If they creep, the handedness assignment or a
`DIR` invert flag is wrong.

---

## 7. Step-pulse frequency conversion

The A4988 is hard-wired to 1/16 microstepping, so:

```
MICROSTEPS_PER_REV = 200 full steps/rev × 16 = 3200 microsteps/rev
```

One revolution is `2π` radians, so the conversion from angular velocity to
pulse rate is a single constant:

```
f_i = ω_i * ( MICROSTEPS_PER_REV / (2*π) )

    = ω_i * ( 3200 / 6.283185307... )

    = ω_i * 509.29581789...              [microsteps/s per rad/s]   ... (7.1)
```

Combining (6) and (7.1) directly, the full input-to-pulse-rate chain is:

```
f_FL = ( vx - vy - k*w ) * 509.295818 / wheel_radius_mm
f_FR = ( vx + vy + k*w ) * 509.295818 / wheel_radius_mm
f_RL = ( vx + vy - k*w ) * 509.295818 / wheel_radius_mm
f_RR = ( vx - vy + k*w ) * 509.295818 / wheel_radius_mm
```

`sign(f_i)` drives the A4988 `DIR` pin. `|f_i|` is the `STEP` pulse rate.

The 509.295818 constant is derived, not measured — it is exact given the
microstepping and motor step angle, and changes only if the microstepping
jumpers or the motor are changed.

---

## 8. Forward kinematics — commanded-velocity estimate

### 8.1 Why a pseudoinverse

The mixing matrix `M` is 4×3. It maps 3 body velocities to 4 wheel speeds, so
it has no ordinary inverse. Recovering body velocity from wheel speeds requires
the **left pseudoinverse** `M⁺ = (MᵀM)⁻¹Mᵀ`, which yields the least-squares
best fit — the body motion most consistent with all four wheel speeds at once.

Because the matrix columns are mutually orthogonal (each column is a ±1 or ±k
pattern with zero dot product against the others), `MᵀM` is diagonal and the
pseudoinverse comes out in closed form with no matrix arithmetic needed:

```
MᵀM = diag( 4/r² ,  4/r² ,  4k²/r² )
```

Inverting the diagonal and multiplying by `Mᵀ` gives:

```
vx = (wheel_radius_mm / 4) * (  ω_FL + ω_FR + ω_RL + ω_RR )
vy = (wheel_radius_mm / 4) * ( -ω_FL + ω_FR + ω_RL - ω_RR )
w  = (wheel_radius_mm / (4*k)) * ( -ω_FL + ω_FR - ω_RL + ω_RR )
```

Substituting `k = (wheelbase_mm + track_width_mm)/2` into the third line to
clear the fraction:

```
w  = (wheel_radius_mm / (2*(wheelbase_mm + track_width_mm)))
                       * ( -ω_FL + ω_FR - ω_RL + ω_RR )
```

Notice the row patterns are exactly the **columns** of the inverse matrix — the
`vy` row carries the strafe sign pattern `(-,+,+,-)` and the `w` row carries the
rotation pattern `(-,+,-,+)`. That symmetry is a direct consequence of column
orthogonality and is a quick way to check the implementation by eye.

Converting from step rates back to angular velocity inverts (7.1):

```
ω_i = f_i * (2*π / 3200) = f_i / 509.295818
```

### 8.2 Round-trip check

Feeding the §11 worked example through IK and then FK reproduces
`(vx, vy, w) = (200.0, 100.0, 0.5)` exactly. Any implementation that fails this
round-trip has a sign error, and the round-trip is worth keeping as a unit test.

---

## 9. Open-loop considerations

### 9.1 This is a command, not a measurement

With no encoders, §8 tells you what the wheels were **asked** to do. Two error
sources separate that from reality, and both are systematic rather than random:

1. **Integer rounding** of `f_i` at the timer/RMT peripheral. The hardware can
   only emit whole pulses at whole timer ticks, so the realised rate is
   quantised. The error is small but *biased* — it accumulates rather than
   averaging out.
2. **Skipped steps** under acceleration or load. Critically, this error is
   always in one direction: the wheel can lose steps but never gain them. Real
   motion is always ≤ commanded, never more.

Both are systematic, which is exactly what a downstream velocity-bias loop can
estimate and absorb. Random noise could not be corrected this way; a consistent
bias can.

### 9.2 The unobservable mode

The 4×3 matrix has a one-dimensional left null space. Solving `nᵀM = 0` for the
three columns `(1,1,1,1)`, `(-1,1,1,-1)` and `(-k,k,-k,k)` gives the unique
direction (up to scale):

```
(+ω_FL + ω_FR - ω_RL - ω_RR)
```

That is the **front pair opposing the rear pair**. It produces **no body motion
at all** — any energy put into this combination goes entirely into roller scrub
and tyre slip.

Check it against the three motion columns: forward `(+,+,+,+)` gives
`1+1-1-1 = 0`; strafe `(-,+,+,-)` gives `-1+1-1+1 = 0`; rotation `(-,+,-,+)`
gives `-1+1+1-1 = 0`. All three body motions are orthogonal to it, as they must
be.

This is genuinely useful as a health metric. Because the IK in §6 never
generates a non-zero value for it, computing it on the commanded step rates
should always yield zero. If it does not, something downstream — a clamp applied
per-wheel, a saturated axis, a wrong invert flag — has corrupted the command,
and the drivetrain is fighting itself.

---

## 10. What lives where

The derivation above is deliberately free of hardware quirks. Keep it that way:

| Concern | Belongs in | Never in |
|---|---|---|
| Motor phase mirroring (left vs right side) | GPIO layer, per-wheel `dir_invert` flag | the mixing matrix |
| RC stick polarity | RC input mapping layer | the mixing matrix |
| Speed limits / saturation | a **common-factor** scale applied to all four wheels | per-wheel clipping |
| Geometry values | `params.json`, passed as function arguments | compile-time constants |

The reason the first row matters: flipping a sign in the mixing matrix to fix a
motor that spins backwards will appear to work for that one motion and will
silently break the FK round-trip and every other motion involving that wheel.

The reason the third row matters: clipping wheels individually **warps the
motion vector**. If FR saturates and the others do not, the commanded strafe
becomes an arc. Scaling all four by one common factor preserves the direction of
motion and only reduces its magnitude.

---

## 11. Worked numeric example — actual OMNIS build

```
wheel_radius_mm = 30
wheelbase_mm    = 223
track_width_mm  = 230

k             = (223 + 230) / 2 = 226.5 mm
STEPS_PER_RAD = 3200 / (2π)     = 509.295818
k / r         = 226.5 / 30      = 7.55  rad/s of wheel speed per rad/s of yaw
```

### Case A — pure forward, `vx = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | +6.66667 | +3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | +6.66667 | +3395.305 |

### Case B — pure strafe left, `vy = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −6.66667 | −3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | −6.66667 | −3395.305 |

Strafe costs the **same** wheel speed as forward motion for the same body speed.
It nonetheless feels weaker, because only the roller-axis component of each
wheel's force reaches the floor — a force effect, not a kinematics error.

### Case C — pure rotation CCW, `w = 1.0 rad/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −7.55000 | −3845.183 |
| FR | +7.55000 | +3845.183 |
| RL | −7.55000 | −3845.183 |
| RR | +7.55000 | +3845.183 |

The magnitude is exactly the `k/r = 7.55` figure noted above.

### Case D — combined, `vx = 200`, `vy = 100`, `w = 0.5`

`k*w = 226.5 × 0.5 = 113.25 mm/s`

| Wheel | Arithmetic | ω (rad/s) | f (µsteps/s) | DIR |
|---|---|---|---|---|
| FL | (200 − 100 − 113.25)/30 | −0.441667 | **−224.939** | reverse |
| FR | (200 + 100 + 113.25)/30 | +13.775000 | **+7015.550** | forward |
| RL | (200 + 100 − 113.25)/30 | +6.225000 | **+3170.366** | forward |
| RR | (200 − 100 + 113.25)/30 | +7.108333 | **+3620.244** | forward |

FK round-trip: `(200.0, 100.0, 0.5)` ✓
Null-space check: `(−224.939) + 7015.550 − 3170.366 − 3620.244 = 0.000` ✓

Two practical observations from this case:

- FR at 7016 µsteps/s is **2.19 rev/s ≈ 131 RPM**. The A4988 handles that pulse
  rate without difficulty, but a NEMA17 on a 12 V rail has fallen well off its
  torque curve by then. Case D sits near the practical ceiling of this build.
- FL at −225 µsteps/s is close to zero and will **chatter across the direction
  reversal** as the command dithers. See the deadband note in the reference
  document.

---

## 12. Summary of results

```
k    = (wheelbase_mm + track_width_mm) / 2

IK:  ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
     ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
     ω_RL = ( vx + vy - k*w ) / wheel_radius_mm
     ω_RR = ( vx - vy + k*w ) / wheel_radius_mm

     f_i  = ω_i * 3200 / (2*π)

FK:  vx = (wheel_radius_mm/4) * (  ω_FL + ω_FR + ω_RL + ω_RR )
     vy = (wheel_radius_mm/4) * ( -ω_FL + ω_FR + ω_RL - ω_RR )
     w  = (wheel_radius_mm / (2*(wheelbase_mm + track_width_mm)))
                            * ( -ω_FL + ω_FR - ω_RL + ω_RR )
```

No numeric geometry constant appears anywhere above. Changing wheel size or
frame dimensions is a `params.json` edit.
