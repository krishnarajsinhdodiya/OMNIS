# Mecanum Kinematics — Full Symbolic Derivation

**Project:** OMNIS · 4-wheel holonomic mecanum platform
**Layout:** **lateral parallel rollers** — left pair alike, right pair mirrored
**Scope:** Complete first-principles derivation of the inverse and forward
kinematics for the OMNIS drivetrain, kept fully symbolic in
`wheel_radius_mm`, `wheelbase_mm`, and `track_width_mm`.

> This file is the *why*. For the equations alone (no derivation), see
> [`mecanum-kinematics-reference.md`](mecanum-kinematics-reference.md).
> For the firmware, see [`mecanum_kinematics.c`](mecanum_kinematics.c) and
> [`mecanum-kinematics-code-explained.md`](mecanum-kinematics-code-explained.md).

> **This document was re-derived in full when the wheel layout changed.** The
> drivetrain is no longer a textbook X-drive. Everything below describes the
> as-built machine; §13 records what the change did and did not move, for
> anyone holding an older printout.

---

## 1. Hardware assumptions

These are the physical facts the derivation rests on. If any of them change,
the affected section is flagged inline.

| Property | Value | Affects |
|---|---|---|
| Wheel type | 60 mm mecanum, rollers at **45°** to the wheel plane | §5 — the 45° is what makes the mixing coefficients clean ±1 |
| Layout | 4 wheels, rectangular, all axles parallel to the body Y axis | §3, §5 |
| **Roller handedness** | **left pair (FL, RL) share one tilt; right pair (FR, RR) share the mirrored tilt** | **§5, §6, §8 — this is the one that changed** |
| Motor | 2-phase NEMA17-class, 200 full steps/rev (1.8°/step) | §7 |
| Driver | A4988, **fixed** 1/16 microstepping (MS1/MS2/MS3 tied HIGH) | §7 |
| Resulting resolution | 200 × 16 = **3200 microsteps/rev** | §7 |
| Feedback | **None** — open-loop steppers, no encoders | §9 |

The handedness row is the change. It was forced by mounting-shaft and motor
constraints, not chosen, and it is why this document exists in a second
edition. Viewed from above with the robot facing up the page:

```
        FL  /                 \  FR
        RL  /                 \  RR
```

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

### Drawing convention for roller tilt

Roller tilt is stated throughout as a `/` or `\` glyph. That glyph is the
**roller's axis of rotation**, projected into the ground plane, seen from
**above**, with the robot facing **up the page** and body **+y to the
page-left**. It is what you actually see looking down at the top roller of an
assembled wheel.

Two conventions are possible and they are mirror images, so this one is fixed
here, once, and §5.3 gives the bench test that confirms it on hardware.

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
           /│           |           │\
          / │           |           │ \    wheelbase_mm
  +y <───/──┼───────────O           │  \   = front-to-back
  (left)/   │                       │   \
       /    │                       │    \
        RL  o───────────────────────o  RR
           /<────── track_width_mm ─────>\
                    (left-to-right)

  the / and \ marks are the roller tilts: both left wheels alike,
  both right wheels alike, mirrored across the centreline
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

Throughout, `L = wheelbase_mm` and `W = track_width_mm`.

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
mechanics and would be identical for any wheel type. **This section is
completely untouched by the layout change** — the rollers enter in §5.

---

## 5. Step 2 — the mecanum roller constraint

This is where the wheel type enters, and where the layout change bites.

A conventional wheel imposes two constraints: it rolls along its heading and
cannot slide sideways. A mecanum wheel relaxes the second one. Its rim carries
free-spinning rollers mounted at 45°, so the contact patch **can** slide freely
in one direction — perpendicular to the roller's own axis.

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
the two mirrored roller tilts:

```
δ = +1   ->  axis along the forward-LEFT  diagonal  ->  drawn "\"
δ = -1   ->  axis along the forward-RIGHT diagonal  ->  drawn "/"
```

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

Equation (5.3) is the general per-wheel result, and it is **layout-independent**:
it holds for an X-drive, for this parallel layout, or for any other assignment
of the four `δ_i`. Everything specific to OMNIS is now in one place — which
four signs go into `δ`. The firmware mirrors that structure exactly:
`MECANUM_DELTA_FL` … `MECANUM_DELTA_RR` in
[`mecanum_kinematics.h`](mecanum_kinematics.h) are the only statement of the
layout, and (5.3) is implemented literally.

### 5.3 Assigning handedness to wheels — the as-built layout

The physical pairing is: **both left wheels share one roller tilt; both right
wheels share the mirrored tilt.** The wheels are `/` on the left and `\` on
the right, which by the convention in §2 and §5.1 gives:

> ### Sign assignment
> **FL and RL take `δ = -1`** — roller axis `â = (1, -1)/√2`, along the
> forward-right diagonal, drawn `/`. These take the **negative** sign on `vy`.
>
> **FR and RR take `δ = +1`** — roller axis `â = (1, +1)/√2`, along the
> forward-left diagonal, drawn `\`. These take the **positive** sign on `vy`.

This is **not** the textbook X-drive, in which the shared tilts run along the
diagonals (FL with RR, FR with RL). The difference is exactly the rear pair:
FL and FR are assigned as they always were, and RL and RR have swapped.
Consequently **every front-wheel equation in this document is unchanged from
the previous edition, and every rear-wheel equation is different.** That is a
useful thing to check when reviewing a port.

#### If the hardware turns out to be mirrored

Two conventions for reading a roller glyph exist and they are mirror images.
This document fixes one (§2) and derives everything from it consistently, but
consistency is not the same as being right about the hardware, and the
difference is not academic: it flips the direction the robot strafes.

**The bench test that settles it, in one command:**

> Command `vx = vy` (equal forward and left, no yaw), wheels off the ground.
>
> - **Both LEFT wheels dead still**, both right wheels driving → handedness is
>   correct as written.
> - **Both RIGHT wheels dead still** instead → the handedness is mirrored.
>   Negate all four `MECANUM_DELTA_*` constants together and rebuild.
> - **A diagonal pair still** (FL and RR, or FR and RL) → the wheels are not in
>   the parallel layout at all; this document does not describe your robot.
> - **Anything else** → a `DIR` invert flag is wrong. Fix plain forward motion
>   before looking at anything here.

Negate all four `δ` **together**. Every sign in the `vy` **and** `w` columns
flips together — they are not independent, because both derive from the same
`δ_i` in (5.3). Do not flip one column by hand, and do not "fix" a wrong strafe
direction with a `dir_invert` flag: that would break forward motion and the FK
round trip along with it.

### 5.4 The yaw coefficient no longer collapses to a single lever arm

The yaw term in (5.3) is `κ_i = δ_i*x_i - y_i`. Substituting positions from §3
and handedness from §5.3:

```
FL:  δ=-1, x=+L/2, y=+W/2   ->  (-1)(+L/2) - (+W/2)  =  -(L + W)/2
FR:  δ=+1, x=+L/2, y=-W/2   ->  (+1)(+L/2) - (-W/2)  =  +(L + W)/2
RL:  δ=-1, x=-L/2, y=+W/2   ->  (-1)(-L/2) - (+W/2)  =  +(L - W)/2
RR:  δ=+1, x=-L/2, y=-W/2   ->  (+1)(-L/2) - (-W/2)  =  -(L - W)/2
```

**This is the heart of the change.** Under an X-drive all four came out at
`±(L+W)/2` — one lever arm, one symbol `k`, four signs. Here the two axles get
**different lever arms**:

```
k = (L + W)/2          front pair, "long" lever               ... (5.4a)
m = (L - W)/2          rear pair,  "short" lever              ... (5.4b)
```

so `κ_FL = -k`, `κ_FR = +k`, `κ_RL = +m`, `κ_RR = -m`.

Three properties of `k` and `m` are worth naming, because they carry most of
what is different about driving this machine:

**(a) The rear lever is small, and on OMNIS it is tiny.** With `L = 223` and
`W = 230`, `k = 226.5 mm` and `m = -3.5 mm`. The front axle's lever is **64.7×**
the rear's, so the front pair supplies **98.5%** of the yaw authority. During a
pure spin the rear wheels barely turn (§11 Case C) — the rotation is absorbed
almost entirely by the rear rollers scrubbing. This is correct behaviour and
not a fault; it is also the reason a slow yaw command will deadband the rear
pair while the front pair still drives.

**(b) The rear lever changes sign at a square chassis.** `m = (L-W)/2` is
negative when the frame is wider than it is long, positive when it is longer,
and exactly zero when `L = W`. OMNIS sits 3.5 mm from that crossover. Nothing
in the mathematics misbehaves there — the term simply passes through zero, and
§8 shows the matrix stays full-rank regardless — but **do not build an
intuition about which way the rear wheels turn during a spin.** It is not a
robust fact about the machine.

**(c) Two identities that do the algebraic work later.** They hold for any
geometry and are what make §8 come out in closed form:

```
k + m = L          (the two levers sum to the wheelbase)
k - m = W          (and differ by the track width)
```

The X-drive's lesson that "only the sum `L + W` matters, never either dimension
alone" is **no longer true**. Both dimensions now appear separately, and the
wheelbase in particular acquires a role it did not have before — §8 finds it
alone in the yaw scale factor.

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
 │ ω_RR │    r    │   1     -1     +m     │   └      ┘
 └      ┘         │                       │
                  │   1     +1     -m     │
                  └                       ┘

        r = wheel_radius_mm
        k = (wheelbase_mm + track_width_mm) / 2
        m = (wheelbase_mm - track_width_mm) / 2
```

Written out:

```
ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
ω_RL = ( vx - vy + m*w ) / wheel_radius_mm
ω_RR = ( vx + vy - m*w ) / wheel_radius_mm
```

The matrix has three columns and four rows: three degrees of freedom driving
four actuators. It is over-determined, which is the formal reason a mecanum
platform can fight itself (§9.2).

### 6.1 Motion-by-motion check

Each row below is obtained by setting one input non-zero and reading off the
matrix column.

| Motion | Input | FL | FR | RL | RR | Split | Notes |
|---|---|---|---|---|---|---|---|
| Forward | `vx > 0` | + | + | + | + | none | all four identical |
| Strafe **left** | `vy > 0` | **−** | **+** | **−** | **+** | **sides** | all four equal magnitude |
| Rotate **CCW** | `w > 0` | **−** | **+** | **−** | **+** | **sides** | front pair 64.7× faster than rear |
| Diagonal fwd-left | `vx = vy > 0` | **0** | + | **0** | + | — | the whole **left side** idles |
| Diagonal fwd-right | `vx = -vy > 0` | + | **0** | + | **0** | — | the whole **right side** idles |

Two things to take from this table.

**Strafe and yaw now share a sign pattern.** Both split along the sides,
`(−,+,−,+)`. Under an X-drive they were cleanly distinguishable — strafe split
along the diagonals, rotation along the sides — and "which pattern is this?"
was enough to identify a motion by eye. Here the two are told apart only by the
*ratio* between the front and rear magnitudes: equal for strafe, 64.7:1 for
yaw. §8 shows this is not merely cosmetic — the two columns are genuinely
non-orthogonal, at 44° rather than 90°.

**The diagonal test is now a side test, and there are two of them.** Commanding
`vx = vy` leaves both left wheels exactly stationary; `vx = -vy` leaves both
right wheels stationary. Running *both* is what distinguishes a correct
handedness assignment from a mirrored one (§5.3) — a mirrored build passes one
and fails the other. Two adjacent wheels standing still while their neighbours
drive is also easier to see across a bench than a diagonal pair was.

---

## 7. Step-pulse frequency conversion

Unaffected by the layout change — this stage knows nothing about rollers.

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
f_RL = ( vx - vy + m*w ) * 509.295818 / wheel_radius_mm
f_RR = ( vx + vy - m*w ) * 509.295818 / wheel_radius_mm
```

`sign(f_i)` drives the A4988 `DIR` pin. `|f_i|` is the `STEP` pulse rate.

The 509.295818 constant is derived, not measured — it is exact given the
microstepping and motor step angle, and changes only if the microstepping
jumpers or the motor are changed.

---

## 8. Forward kinematics — commanded-velocity estimate

### 8.1 Why a pseudoinverse, and why it got harder

The mixing matrix `M` is 4×3. It maps 3 body velocities to 4 wheel speeds, so
it has no ordinary inverse. Recovering body velocity from wheel speeds requires
the **left pseudoinverse** `M⁺ = (MᵀM)⁻¹Mᵀ`, which yields the least-squares
best fit — the body motion most consistent with all four wheel speeds at once.

Under the X-drive this was nearly free: the three columns were mutually
orthogonal, `MᵀM` was diagonal, and `M⁺` came out as a scaled transpose. **That
shortcut is gone.** Writing the columns as

```
c_vx = (1, 1, 1, 1)/r
c_vy = (-1, +1, -1, +1)/r
c_w  = (-k, +k, +m, -m)/r
```

the `vx` column is still orthogonal to both others — `c_vx · c_vy = 0` and
`c_vx · c_w = -k + k + m - m = 0` — but the other pair is not:

```
c_vy · c_w  =  [ (-1)(-k) + (+1)(+k) + (-1)(+m) + (+1)(-m) ] / r²

            =  2(k - m) / r²  =  2W / r²                        ... (8.1)
```

using the identity `k - m = W` from §5.4(c). **The strafe and yaw columns
overlap by exactly twice the track width**, and that is the algebraic signature
of the whole layout change. It vanishes only for `W = 0`, which is not a robot.

So:

```
        ┌                          ┐
        │  4      0        0       │
MᵀM  =  │  0      4       2W       │ / r²
        │  0     2W    2(k² + m²)  │
        └                          ┘
```

and `2(k² + m²) = L² + W²`, since `k² + m² = [(L+W)² + (L-W)²]/4 = (L² + W²)/2`.

### 8.2 Inverting the 2×2 block

The `vx` row decouples immediately; the `vy` and `w` rows need the 2×2 block

```
        ┌            ┐
   B =  │  4     2W  │          det B = 4(L² + W²) - 4W² = 4L²     ... (8.2)
        │ 2W   L²+W² │
        └            ┘
```

**The determinant is `4L²`** — the track width cancels completely. Three
consequences follow directly, and they are the most useful results in this
document:

1. **The layout is holonomic for every real robot.** `det B = 0` only if
   `L = 0`. There is no chassis aspect ratio, square included, at which this
   drivetrain loses a degree of freedom. The near-singular look of a square
   frame (§5.4b, where `m → 0`) is an illusion: the rear pair stops
   contributing to *yaw*, but it still contributes to `vy`, and rank is
   preserved.
2. **Yaw resolution is set by the wheelbase alone.** Not by `L + W`, as under
   the X-drive. A longer robot estimates its own rotation better; a wider one
   does not.
3. It is why §8.3 comes out in closed form at all, with no matrix arithmetic
   at runtime.

Inverting and multiplying by `Mᵀ` (the algebra is mechanical; the result was
checked against an exact rational pseudoinverse — see §8.4):

### 8.3 The result

State it in terms of the two **axle differentials**, which is where it is
cleanest:

```
Δ_F = ω_FL - ω_FR          front differential
Δ_R = ω_RL - ω_RR          rear  differential
```

```
vx = (wheel_radius_mm / 4) * ( ω_FL + ω_FR + ω_RL + ω_RR )

vy = (wheel_radius_mm / (4*wheelbase_mm))
            * ( (track_width_mm - wheelbase_mm) * Δ_F
              - (track_width_mm + wheelbase_mm) * Δ_R )

w  = (wheel_radius_mm / (2*wheelbase_mm)) * ( -Δ_F + Δ_R )
```

Or fully expanded, one coefficient per wheel:

```
vy = (r / (4L)) * ( (W-L)*ω_FL - (W-L)*ω_FR - (L+W)*ω_RL + (L+W)*ω_RR )

w  = (r / (2L)) * (   -ω_FL    +     ω_FR   +     ω_RL   -      ω_RR )
```

Read the shape of it:

> **Yaw is the difference of the two axle differentials. Strafe is their
> weighted sum.**

Under the X-drive it was the other way round — yaw came from the side-sum
difference and strafe from the diagonal pattern. The two forms look alike
enough that swapping them is the single most likely error when porting older
code, and it is silent: forward motion still works perfectly, and only strafe
and yaw come out interchanged. The round-trip test in §8.4 catches it
immediately, which is why it is worth keeping as a unit test.

Also note what is **not** in the `w` row: `track_width_mm`. The yaw estimate is
built from the wheelbase alone, per §8.2(2).

Converting from step rates back to angular velocity inverts (7.1):

```
ω_i = f_i * (2*π / 3200) = f_i / 509.295818
```

### 8.4 Verification

Two independent checks, both mechanical rather than by eye:

- **Against an exact pseudoinverse.** The closed form above was compared with
  `(MᵀM)⁻¹Mᵀ` computed in exact rational arithmetic, for **both** handedness
  signs and four geometries (223×230, 300×200, 250×250 square, 100×400). Every
  coefficient matched exactly, with no floating point in the comparison.
- **Round trip.** Feeding the §11 worked example through IK and then FK
  reproduces `(vx, vy, w) = (200.0, 100.0, 0.5)`. Any sign error fails this, and
  `test/test_kinematics.c` runs it on every build along with a per-axis round
  trip for `vx`, `vy` and `w` separately — three separate tests, because a
  rank-deficient layout would fail only one of them.

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

### 9.2 The unobservable mode — unchanged by the layout

The 4×3 matrix has a one-dimensional left null space. Solving `nᵀM = 0`:

- `n · c_vx = 0` and `n · c_vy = 0` together force `n₃ = -n₁` and `n₄ = -n₂`.
- `n · c_w = 0` then reduces to `(k - m)(n₁ - n₂) = W(n₁ - n₂) = 0`, so
  `n₁ = n₂`.

giving the unique direction (up to scale):

```
(+ω_FL + ω_FR - ω_RL - ω_RR)
```

That is the **front pair opposing the rear pair**. It produces **no body motion
at all** — any energy put into this combination goes entirely into roller scrub
and tyre slip.

**This is identical to the X-drive's null space**, and that is not a
coincidence worth shrugging at: the derivation above used only `k ≠ m`, i.e.
`W ≠ 0`. The same vector annihilates all three columns **for either handedness
and for any geometry**. Of everything in this document it is the one result the
layout change left completely alone, so `mecanum_null_space()` needed no edit
and the health metric keeps its meaning.

Check it against the three motion columns: forward `(+,+,+,+)` gives
`1+1-1-1 = 0`; strafe `(-,+,-,+)` gives `-1+1+1-1 = 0`; rotation `(-k,+k,+m,-m)`
gives `-k+k-m+m = 0`. All three body motions are orthogonal to it, as they must
be.

Because the IK in §6 never generates a non-zero value for it, computing it on
the commanded step rates should always yield zero. If it does not, something
downstream — a clamp applied per-wheel, a saturated axis, a wrong invert flag —
has corrupted the command, and the drivetrain is fighting itself.

### 9.3 What the layout costs: conditioning

This section has no X-drive counterpart, because under the X-drive there was
nothing to say: orthogonal columns cost nothing.

**The strafe and yaw columns meet at 44°, not 90°.** From (8.1):

```
cos θ = (c_vy · c_w) / (|c_vy| |c_w|)
      = 2W / (2 * sqrt(L² + W²))
      = W / sqrt(L² + W²)
      = 230 / 320.36 = 0.7179        ->  θ = 44.1°
```

Note this depends only on the frame's aspect ratio, not on the wheels.

**What that does and does not affect.** It does *not* affect the IK: any
`(vx, vy, w)` is still reachable exactly, and — worth stating plainly, because
it is the first thing people assume was lost — **the peak wheel rate per unit
of commanded motion is unchanged on all three axes.** The same step-rate ceiling
buys the same top speed, the same strafe speed, and the same yaw rate as before.
The layout costs nothing in performance.

What it costs is **precision in the estimate**. The least-squares solution
amplifies per-wheel error more than an orthogonal one would. Taking independent
per-wheel error `σ` and propagating through the §8.3 rows:

| Estimate | X-drive error gain | This layout | Ratio |
|---|---|---|---|
| `vx` | 15.00 | 15.00 | **1.00×** |
| `vy` | 15.00 | 21.55 | **1.44×** |
| `w` | 0.0662 | 0.1345 | **2.03×** |

So dead reckoning drifts about 1.4× faster laterally and about 2× faster in
heading, for the same stepper behaviour. On a robot with no encoders and a
§9.1 error budget that is already dominated by skipped steps, this is a real
but second-order cost — worth knowing before trusting a long open-loop
trajectory, not worth redesigning the chassis over.

**Where it does bite harder: disturbance rejection.** Two wheel patterns that
are 44° apart are, by that much, harder to tell apart. A wheel-speed error
whose shape resembles the strafe pattern will be partly read as yaw and vice
versa. There is no estimator here to be fooled today, but if one is ever added,
this is the coupling it will have to deal with — and the reason `L` appears
alone in §8.3 is worth remembering there: **a longer wheelbase directly improves
yaw conditioning.**

---

## 10. What lives where

The derivation above is deliberately free of hardware quirks. Keep it that way:

| Concern | Belongs in | Never in |
|---|---|---|
| Roller handedness | the four `MECANUM_DELTA_*` constants, negated **together** | anywhere else |
| Motor phase mirroring (left vs right side) | GPIO layer, per-wheel `dir_invert` flag | the mixing matrix |
| RC stick polarity | RC input mapping layer | the mixing matrix |
| Speed limits / saturation | a **common-factor** scale applied to all four wheels | per-wheel clipping |
| Geometry values | `params.json`, passed as function arguments | compile-time constants |

The reason the first row matters: the handedness constants and the yaw levers
are not independent quantities that happen to agree — the levers are *computed
from* the handedness, in `mecanum_yaw_levers()`. Changing one without the other
is not expressible in this code, which is the point.

The reason the second row matters: flipping a sign in the mixing matrix to fix a
motor that spins backwards will appear to work for that one motion and will
silently break the FK round-trip and every other motion involving that wheel.

The reason the fourth row matters: clipping wheels individually **warps the
motion vector**. If FR saturates and the others do not, the commanded strafe
becomes an arc. Scaling all four by one common factor preserves the direction of
motion and only reduces its magnitude.

---

## 11. Worked numeric example — actual OMNIS build

```
wheel_radius_mm = 30
wheelbase_mm    = 223
track_width_mm  = 230

k             = (223 + 230) / 2 =  226.5 mm      front yaw lever
m             = (223 - 230) / 2 =   -3.5 mm      rear  yaw lever
|k / m|                         =  64.71         front:rear yaw authority
STEPS_PER_RAD = 3200 / (2π)     = 509.295818
k / r         = 226.5 / 30      = 7.55   rad/s of front-wheel speed per rad/s of yaw
m / r         =  -3.5 / 30      = -0.1167 rad/s of rear-wheel speed per rad/s of yaw
```

### Case A — pure forward, `vx = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | +6.66667 | +3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | +6.66667 | +3395.305 |
| RR | +6.66667 | +3395.305 |

Identical to the X-drive: forward motion never involved the rollers' handedness.

### Case B — pure strafe left, `vy = 200 mm/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −6.66667 | −3395.305 |
| FR | +6.66667 | +3395.305 |
| RL | −6.66667 | −3395.305 |
| RR | +6.66667 | +3395.305 |

Left side back, right side forward — the pattern that would be a *spin* on a
differential-drive robot, and was a *diagonal* pattern under the X-drive. All
four magnitudes are equal, which is what distinguishes this from Case C.

Strafe costs the **same** wheel speed as forward motion for the same body speed,
exactly as before. It nonetheless feels weaker, because only the roller-axis
component of each wheel's force reaches the floor — a force effect, not a
kinematics error.

### Case C — pure rotation CCW, `w = 1.0 rad/s`

| Wheel | ω (rad/s) | f (µsteps/s) |
|---|---|---|
| FL | −7.55000 | −3845.183 |
| FR | +7.55000 | +3845.183 |
| RL | **−0.11667** | **−59.418** |
| RR | **+0.11667** | **+59.418** |

**The signature of this layout.** The front wheels spin at `k/r = 7.55 rad/s`
as they always did; the rear wheels crawl at 1.5% of that. If a bench test shows
all four at 3845, the build is still running X-drive kinematics.

At the usual 20 steps/s deadband, the rear pair is held stopped for any
commanded yaw below about **0.34 rad/s** while the front pair drives normally.
That is correct, and worth writing on the bench card before someone reports it
as a dead motor.

### Case D — combined, `vx = 200`, `vy = 100`, `w = 0.5`

`k*w = +113.25 mm/s`,  `m*w = -1.75 mm/s`

| Wheel | Arithmetic | ω (rad/s) | f (µsteps/s) | DIR |
|---|---|---|---|---|
| FL | (200 − 100 − 113.25)/30 | −0.441667 | **−224.939** | reverse |
| FR | (200 + 100 + 113.25)/30 | +13.775000 | **+7015.550** | forward |
| RL | (200 − 100 − 1.75)/30 | +3.275000 | **+1667.944** | forward |
| RR | (200 + 100 + 1.75)/30 | +10.058333 | **+5122.667** | forward |

FK round-trip: `(200.0, 100.0, 0.5)` ✓
Null-space check: `(−224.939) + 7015.550 − 1667.944 − 5122.667 = 0.000` ✓

Three practical observations from this case:

- FR at 7016 µsteps/s is **2.19 rev/s ≈ 131 RPM**. The A4988 handles that pulse
  rate without difficulty, but a NEMA17 on a 12 V rail has fallen well off its
  torque curve by then. Case D sits near the practical ceiling of this build.
  Unchanged from the X-drive: the peak wheel is a front wheel either way.
- FL at −225 µsteps/s is close to zero and will **chatter across the direction
  reversal** as the command dithers. See the deadband note in the reference
  document.
- The **rear pair spread is much narrower than the front pair's** (1668 to 5123
  against −225 to 7016), because only the front pair carries the yaw term in any
  meaningful amount. Under an X-drive the two axles saturated together; here the
  front pair reaches the clamp first whenever yaw is commanded, and the
  common-factor clamp then scales the whole command.

### Case E — the handedness bench test

`vx = vy = 200 mm/s` (diagonal forward-left):

| Wheel | f (µsteps/s) |
|---|---|
| FL | **0.000** |
| FR | +6790.611 |
| RL | **0.000** |
| RR | +6790.611 |

and its mirror, `vx = 200`, `vy = -200` (diagonal forward-right), idles FR and
RR instead. Run **both** — see §5.3.

---

## 12. Summary of results

```
k    = (wheelbase_mm + track_width_mm) / 2        front yaw lever
m    = (wheelbase_mm - track_width_mm) / 2        rear  yaw lever

IK:  ω_FL = ( vx - vy - k*w ) / wheel_radius_mm
     ω_FR = ( vx + vy + k*w ) / wheel_radius_mm
     ω_RL = ( vx - vy + m*w ) / wheel_radius_mm
     ω_RR = ( vx + vy - m*w ) / wheel_radius_mm

     f_i  = ω_i * 3200 / (2*π)

FK:  Δ_F = ω_FL - ω_FR        Δ_R = ω_RL - ω_RR

     vx = (wheel_radius_mm / 4) * ( ω_FL + ω_FR + ω_RL + ω_RR )

     vy = (wheel_radius_mm / (4*wheelbase_mm))
                * ( (track_width_mm - wheelbase_mm) * Δ_F
                  - (track_width_mm + wheelbase_mm) * Δ_R )

     w  = (wheel_radius_mm / (2*wheelbase_mm)) * ( -Δ_F + Δ_R )

Null space (unchanged): ( +ω_FL + ω_FR - ω_RL - ω_RR ) = 0
```

No numeric geometry constant appears anywhere above. Changing wheel size or
frame dimensions is a `params.json` edit.

---

## 13. What the layout change moved — a reviewer's checklist

For anyone holding a printout of the previous (X-drive) edition, or reviewing
the port.

### Changed

| | Was (X-drive) | Now (parallel rollers) |
|---|---|---|
| Shared roller tilt | FL+RR, and FR+RL | **FL+RL, and FR+RR** |
| `δ` assignment | FL −1, FR +1, RL **+1**, RR **−1** | FL −1, FR +1, RL **−1**, RR **+1** |
| Yaw lever arms | `±k` on all four | **`±k` front, `±m` rear** — 64.7:1 |
| Rear-wheel IK rows | `vx + vy ∓ k*w` | **`vx − vy ± m*w`** |
| Strafe pattern | diagonals `(−,+,+,−)` | **sides `(−,+,−,+)`** |
| Bench idle test | `vx = vy` idles FL **and RR** | **idles the whole LEFT SIDE** |
| `MᵀM` | diagonal | **not diagonal** — `c_vy · c_w = 2W` |
| FK yaw scale | `r / (2(L+W))` | **`r / (2L)`** — track width drops out |
| FK structure | yaw ← side pattern, strafe ← diagonal pattern | **yaw ← Δ_F − Δ_R, strafe ← weighted Δ_F + Δ_R** |
| Odometry error gain | baseline | **1.44× on `vy`, 2.03× on `w`** |

### Unchanged

| | |
|---|---|
| Body frame, sign conventions, wheel positions | §2, §3 |
| Rigid-body contact-patch velocity (4.1) | §4 |
| The general per-wheel constraint (5.3) | §5 — only the four `δ` differ |
| Front-wheel IK rows | `vx ∓ vy ∓ k*w` exactly as before |
| Forward motion, and the `vx` FK row | all four wheels equal; `(r/4)·Σω` |
| Step-rate conversion, `STEPS_PER_RAD` | §7 |
| **Null space `(+1,+1,−1,−1)`** | §9.2 — and hence the slip metric |
| Peak wheel rate per unit motion | §9.3 — same top speed, strafe and yaw rate |
| Holonomy | §8.2 — rank 3 for any non-zero wheelbase |
| Clamp, deadband and pipeline ordering rules | §10 |
