---
name: Minecraft26.2-IceBoat-Physics_Engineer
description: >
  Engineering protocol for any Minecraft 26.2 boat dynamics model, predictor, controller, planner or
  simulator — especially ice boats (blue ice / packed ice racing), where the boat has no lateral grip
  and every turn is drift. Contains the complete 26.2 vanilla boat model (tick order, damping, control
  inputs, friction table) plus the slip-angle/rotation-memory math that a naive "car with a steering
  angle" model gets wrong. Use whenever the task touches Turneler's navigation package, BezierPlanner,
  BoatSimulator, BoatDynamics, BoatKinematics, PurePursuitController, MppiPlanner, DeepLearn training,
  or any Minecraft boat physics/prediction/autopilot code: "ice boat", "boat drift", "slip angle",
  "turn radius on ice", "boat dynamics", "boat prediction", "boat physics 26.2", "controlBoat",
  "floatBoat", "MPPI boat", "ice retention", "blue ice autopilot". Do NOT use for non-Minecraft
  vehicle physics or for pure UI work.
---

# Minecraft 26.2 Ice-Boat Physics Engineer

Ice boats are drift machines. You cannot steer one with a heading error: A/D are signed angular
**acceleration**, momentum is a damped world vector that never rotates with the bow, and there is
**no lateral friction at all**. Every yaw change you plan is paid for by thrust pointing partly
sideways, and every key you release leaves rotation that keeps happening. Build the model around
the slip angle, or the model will plan corners the boat cannot take.

Reference implementation for this repository: `D:\Turneler`. Authority document:
`docs/boat-physics-26.2.md`. Vanilla source of truth: `AbstractBoat` (26.2).

---

## 1. The complete 26.2 boat model

`AbstractBoat.tick()`, locally-controlled boat (`isLocalInstanceAuthoritative()`):

```text
oldStatus  = status
status     = getStatus()                 // re-samples ground friction every tick
if status is not UNDER_WATER/UNDER_FLOWING_WATER: outOfControlTicks = 0 else ++
                                             // server ejects passengers at outOfControlTicks >= 60

super.tick(); interpolation.interpolate()
if isLocalInstanceAuthoritative():
    if firstPassenger is not Player: setPaddleState(false, false)
    floatBoat()                          // <-- damping happens HERE
    if level.isClientSide():             // client decides, server is told the paddles
        controlBoat()                    // <-- yaw + thrust happen HERE
        sendPacketToServer(ServerboundPaddleBoatPacket(paddle[0], paddle[1]))
    move(MoverType.SELF, deltaMovement)  // <-- position integration LAST
else:
    setDeltaMovement(ZERO)
applyEffectsFromBlocks() twice; tickBubbleColumn(); paddles; pushable-entity scan
```

**Order is load-bearing: `getStatus` → `floatBoat` (damp) → `controlBoat` (rotate, then thrust) →
`move`.** Turneler's `BoatControlMixin` injects at `HEAD` of `controlBoat`, i.e. *after* this tick's
damping and *before* this tick's yaw step.

### 1.1 Status and ground friction

```text
getStatus():
    if isUnderwater() != null:  waterLevel = bbox.maxY; return it (UNDER_WATER / UNDER_FLOWING_WATER)
    if checkInWater():          return IN_WATER
    f = getGroundFriction()
    if f > 0.0F:                landFriction = f; return ON_LAND
    else:                       return IN_AIR          // NaN > 0 is FALSE, so "no contact" -> IN_AIR

getGroundFriction():
    box   = [bb.minX, bb.minY - 0.001, bb.minZ] .. [bb.maxX, bb.minY, bb.maxZ]   // hull-bottom slice
    shape = Shapes.create(box);  sum = 0;  count = 0
    for x in floor(minX)-1 .. ceil(maxX)+1, z likewise, y in floor(minY)-1 .. ceil(maxY)+1:
        skip the x/z border rules ("edges" != 2 filter, interior y only when edges <= 0)
        skip LilyPadBlock
        if blockState collision shape (moved to pos) ∧ shape is not empty:
            sum += block.getFriction(); count++
    return sum / count            // count == 0 -> NaN -> IN_AIR
```

Friction is a **retention multiplier**: larger = less speed and rotation lost per tick.

| Block | `getFriction` | per-tick retention | terminal speed `a·F/(1-F)` | speed time const `1/(1-F)` |
| --- | --- | --- | --- | --- |
| Regular / packed / frosted ice | 0.98 | 0.98 | 1.96 blocks/tick | 50 ticks (2.5 s) |
| Blue ice | 0.989 | 0.989 | 3.60 blocks/tick | 90.9 ticks (4.5 s) |
| (soul sand, slime etc.) | other | — | — | — |

`landFriction` is **re-set by `getStatus()` every tick**, so the player-specific
`landFriction /= 2.0F` inside `floatBoat` does **not** double the damping: this tick already copied
the un-halved value into `invFriction`. Do not model `/2` as a per-tick factor.

### 1.2 `floatBoat()` — damping

```text
vspeed = -gravity            // AbstractBoat.getDefaultGravity() == 0.04
buoyancy = 0.0
invFriction = 0.05F

if oldStatus == IN_AIR and status not in {IN_AIR, ON_LAND}:     // splash landing only
    waterLevel = getY(1.0)
    targetY = getWaterLevelAbove() - bbHeight + 0.101
    if noCollision(bbox moved to targetY): setPos(y = targetY); deltaMovement.y = 0; lastYd = 0
    status = IN_WATER
else:
    IN_WATER            -> buoyancy = (waterLevel - getY()) / bbHeight; invFriction = 0.9
    UNDER_FLOWING_WATER -> vspeed = -7.0E-4;                             invFriction = 0.9
    UNDER_WATER         -> buoyancy = 0.01;                              invFriction = 0.45
    IN_AIR              ->                                             invFriction = 0.9
    ON_LAND             -> invFriction = landFriction;
                           if controlling passenger is Player: landFriction /= 2.0F  (next tick)
                           // absent controlling player the /2 never runs

    deltaMovement = (mx * invFriction, my + vspeed, mz * invFriction)   // X/Z scaled, Y not
    deltaRotation *= invFriction                                        // spin damped too
    if buoyancy > 0: deltaMovement.y = (my + buoyancy * (0.04 / 0.65)) * 0.75
```

**On-land summary — the model that matters:**

```text
v      <- v * F                     (F = 0.98 ice, 0.989 blue ice)
omega  <- omega * F                 (deltaRotation, degrees per tick)
```

### 1.3 `controlBoat()` — yaw, then thrust

```text
if !isVehicle(): return
acceleration = 0.0F
if inputLeft:  deltaRotation -= 1.0F          // 1 degree/tick per tick of angular acceleration
if inputRight: deltaRotation += 1.0F
if (inputRight != inputLeft) && !inputUp && !inputDown:
    acceleration += 0.005F                    // bare turn: only with neither W nor S
setYRot(getYRot() + deltaRotation)            // yaw integrated BEFORE thrust
if inputUp:   acceleration += 0.04F
if inputDown: acceleration -= 0.005F
deltaMovement += ( sin(-yaw_deg * PI/180) * acceleration, 0.0,
                    cos( yaw_deg * PI/180) * acceleration )   // thrust along the NEW yaw
setPaddleState(...)                           // cosmetic/animation only
```

**Existing X/Z momentum is never rotated to the new yaw.** Yaw is `float` degrees; vanilla uses
`Mth.sin/cos` on float, so double-precision predictions will differ in the last bits.

### 1.4 The six keys, in model terms

| Keys | Linear effect | Angular effect |
| --- | --- | --- |
| W | `+0.04` along bow | none |
| S | `-0.005` along bow | none |
| A | `+0.005` along bow (only without W/S) | `-1 °/tick` |
| D | `+0.005` along bow (only without W/S) | `+1 °/tick` |
| W+A / W+D | `+0.04` along bow | `∓1 °/tick` |
| released | none | decays `omega *= F` (does **not** stop rotation) |

W+A and W+D are the effective turning actions: W supplies the acceleration that redirects existing
drift while A/D change angular velocity. A or D alone cannot cancel or redirect momentum.

### 1.5 Minimal reference loop (exact)

```kotlin
// state = { x, z, vx, vz, yaw, omega }  — after floatBoat damping, before controlBoat inputs
fun tick(s: State, F: Double, w: Boolean, sKey: Boolean, a: Boolean, d: Boolean) {
    // ... floatBoat: damp first
    vx *= F; vz *= F; omega *= F
    // ... controlBoat
    val steer = (if (d) 1.0 else 0.0) - (if (a) 1.0 else 0.0)
    omega += steer                       // degrees per tick^2
    yaw += omega                         // degrees; wrapping is vanilla's
    var acc = 0.0
    if (a != d && !w && !sKey) acc += 0.005
    if (w) acc += 0.04
    if (sKey) acc -= 0.005
    val r = Math.toRadians(yaw)
    vx += -sin(r) * acc                  // thrust along the NEW heading
    vz +=  cos(r) * acc
    // ... move(MoverType.SELF, deltaMovement)
    x += vx; z += vz
    // NOTE: no lateral term, no grip term, no speed-along-bow assumption anywhere.
}
```

Turneler's `BoatDynamics.move/damp` + `BoatKinematics.step` (in
`src/main/kotlin/adreno/turneler/navigation/BoatDynamics.kt`) are exactly this, using
`BoatSimulator.FORWARD_ACCELERATION = 0.04`, `REVERSE_ACCELERATION = 0.005`,
`TURN_ACCELERATION = toRadians(1.0)`. Keep the pair in sync — `MppiTest.scalarRolloutMirrorsTheRecordedDynamics`
pins them against each other.

---

## 2. Drift: the doctrine

### 2.1 Momentum has no grip

With `F` near 1 there is nothing that resists sideways motion. `v` is a free world vector scaled by a
scalar; the bow direction only enters through the thrust vector. Consequences:

- **Heading is nearly free; momentum is expensive.** You can spin the bow at up to
  `omega_ss = 1·F/(1-F)` ≈ **49 °/tick on ice**, ≈ 90 °/tick on blue ice. Redirecting the *velocity*
  is bounded by `0.04` per tick.
- Slip angle `beta = angle(bow, v)` is a **first-class state variable**, not noise. Every turn begins
  by pointing the bow inside the curve and letting momentum catch up.
- Lateral acceleration available is `a_lat = 0.04·sin(beta)`. The boat will not travel on a curve
  tighter than

  ```text
  omega_required = v / r        =>   v_max(r) = sqrt(0.04 · r) = 0.2 · sqrt(r)
  r_min(v)      = v² / 0.04
  ```

  radius 36 → 1.2 blocks/tick (the default cruise speed), radius 50 → 1.41, radius 100 → 2.0.
  **Above that, no plan can hold the line: the boat must slow down or run wide.** A radius-50 bend is
  clean at cruise `1.2` and is left on the outside at the 2.0 the throttle reaches when it is never
  eased (`docs/boat-physics-26.2.md`, MPPI section).
- Slip is **free of scrub cost**: there is no lateral friction, so a large slip angle costs no speed —
  only the forward acceleration you gave up by pointing the bow away from the velocity.
- Braking: natural ice drag is `v·(1-F)` ≈ `0.02v` (= 0.024/tick at cruise 1.2), which is **five times
  the whole S key** (0.005). On ice, releasing W brakes harder than pressing S. Stopping from 1.96
  blocks/tick by drag alone takes ≈ 115 ticks. Do not plan as if S is a brake.

### 2.2 Rotation memory (the other half of drift)

`omega` decays by `F` per tick but is **never zeroed**:

- Releasing A/D from the steady spin `omega_ss = 49 °/tick` adds a residual
  `omega_ss · F/(1-F)` ≈ **2400° ≈ 6.7 full turns** of continued rotation over the following ~50 ticks.
  Releasing the key is not "stop turning".
- Consequence for controllers: a residual spin must be **counter-steered**, not waited out. If the
  measured `deltaRotation` has the same sign as the correction you want, the first action must oppose
  it — otherwise the same-sign input stacks (`NavigationTest.controllerBrakesExistingIceSpinInsteadOfStackingSameSignInput`).
- Consequence for planners: yaw is a double integrator with a long tail, so a heading-error P-controller
  oscillates at speed. Plan *torque sequences*, or plan with `SteeringDynamics.yawDelta(...)`
  (closed form of `omega·f(1-fⁿ)/(1-f)` plus the input response term) when you need an analytic
  "which input for N ticks reaches this heading".
- Blue ice nearly doubles both time constants (`1/(1-F)`: 50 → 91 ticks). Anything tuned on regular
  ice must be re-checked at 0.989, and vice versa.

### 2.3 What drift demands of a model

A model that cannot represent all four of these is wrong for ice and will plan corners the boat
cannot take:

1. velocity as a **world 2-D vector** (not `speed · facing`);
2. yaw/heading as an independent variable from travel direction (slip must be representable);
3. angular velocity as a **state** with decay, not a computed steering output;
4. damping of both `v` and `omega` by the friction of the **current** position, sampled per step
   (ice coverage changes along the route).

### 2.4 Design rules for any dynamics code, planner, or trainer here

- **Never rotate momentum into the new heading.** `v` is not `speed * facing`; turning the bow must
  leave `v` untouched this tick. `BoatPredictionTest.turningTheBowDoesNotRotateOldMomentum` pins it.
- **Never damp twice.** The mixin runs after this tick's damping. The first predicted step starts
  from an already-damped measured state; only steps *after* the anchor damp.
- **Never model steering as a steering angle or as mutually exclusive W/A/D.** A/D are signed angular
  acceleration; W+A and W+D must exist in every controller, simulator, dataset label and inference
  path. Dropping them removes the only way to redirect momentum.
- **Never treat friction as a per-tick `/2`** because of the player branch in `floatBoat`.
- **Never let released keys mean "no rotation"** — they mean `omega *= F`, a long tail.
- **Never assume a curvature is followable at current speed.** Gate curvature by
  `sqrt(0.04 / curvature)` (lateral limit) and by the angular limit
  `TURN_ACCELERATION / (1 - F) / curvature`, as `BoatSimulator.assess` does, then propagate a braking
  envelope backwards from corners.
- **Keep a speed envelope / easing knob.** Above cruise, thrust must ease off (Turneler:
  `LIFT_OFF_BAND = 0.2`, continuous rather than a branch, because a float32 GPU batch and the double
  solver would otherwise flip on different ticks). Hardware-like tuning knobs — cruise limit, beam,
  re-decision interval — stay configurable; a model that only works at one speed is a model that has
  not been calibrated.
- **Collisions: reject, don't slide.** Predictions that would clip a non-ice/colliding cell are
  discarded (`blocked`) with a large cost; there is no wall-slide model. Keep it that way and say so.
- **Re-anchor every segment.** Predictions are consumed one segment at a time and re-planned from
  measured state; never integrate open-loop across a decision boundary.
- **Prefer the scalar rollout (`BoatKinematics`) for hot loops** and keep it mirrored against the
  record-based `BoatDynamics`; a rollout that drifts from the record model is a silent planner bug.
- **Rank candidates with a heading-aware term.** Ranking purely by summed speed fills the beam with
  straight lines that forget how to turn (a held turn is genuinely slower because its thrust points
  away from the velocity). Turneler splits the tier over eight headings for this reason.

---

## 3. Failure signatures → cause

| Symptom | Cause |
| --- | --- |
| Plan says "turn here", boat slides off the ice wide | planner used a car-like radius, ignored `v²/0.04`; corner too tight for speed |
| Boat oscillates, weaves down a straight | heading P-controller on a double integrator with 50–90 tick rotation tail |
| Boat spins in circles after the corner | residual `omega *= F` ignored; not counter-steered |
| Planner keeps choosing straight/full-W lines | ranking by speed only, no heading diversity in the beam |
| Predictions diverge from measured states over a ride | model stopped describing the boat: check damping order, `/2` misuse, per-step friction sampling, yaw units — replay an `mppi-trace.bin` |
| Boat is slow to react at speed | needing `0.04·sin(beta)`: slip must be established *before* the corner, i.e. broader turn lead |
| Turning "does nothing" | used A/D alone; the bare-turn thrust is 0.005 = 1/8 of W |
| Corner is left on the outside wide open | cruise limit above `0.2·sqrt(radius)`; no easing band |

---

## 4. Where behavior lives (Turneler)

| File | Role |
| --- | --- |
| `navigation/BoatDynamics.kt` | tick model (`damp`, `move`) + `BoatKinematics` scalar mirror |
| `navigation/BoatSimulator.kt` | constants (`0.04`, `0.005`, `1°`), closed-loop forward simulation, speed envelope `assess` |
| `navigation/SteeringDynamics.kt` | closed-form yaw delta / input for a target heading over N ticks |
| `navigation/BezierPlanner.kt`, `PredictivePilot.kt` | candidate curves, receding horizon |
| `navigation/MppiPlanner.kt` | six-key search, heading-bucketed beam cut, easing, interval from `clearance()` |
| `navigation/PurePursuitController.kt`, `DutyCycle` | path → `BoatInput`, fractional effort |
| `navigation/TerrainConfidenceSampler.kt` | perception weights (`clamp(mean*1.2,0,1)`, `MIN_CONFIDENCE = 0.2`) — **not** a substitute for block friction |
| `client/BoatPilot.kt`, `MinecraftTerrain.kt` | live runtime, terrain adapter, MPPI recorder |
| `src/client/.../mixin/BoatControlMixin.java` | the `controlBoat` HEAD hook |

Invariants: nothing under `adreno.turneler.navigation` may import `net.minecraft.*`/`net.fabricmc.*`
(world access only through `NavigationEnvironment`); mixin handlers are prefixed `turneler$`; logging
via SLF4J; `README.md` "Current behavior" and `docs/boat-physics-26.2.md` stay in sync when model
boundaries change. Model scope is horizontal on-land only — water transitions, vertical steps,
entities and bubble columns are not modeled; yaw is float in vanilla, so last-bit differences between
double and float32 evaluations are expected and must be absorbed with an epsilon, not by loosening
physics tolerances.

## 5. Verify like this

1. `./gradlew test` (JDK 25, `--offline` if needed). Baseline: 26 tests, 2 pre-existing failures
   (`BoatPredictionTest.highSpeedBendsAnticipateAndUseForwardThrustToRedirectMomentum`,
   `NavigationTest.closedLoopAlternatingCurvatureStaysOnTheIce`) — confirm a failure is new before
   blaming your change.
2. Model-level: `BoatPredictionTest` pins tick order, all 16 input combinations, both ice retentions,
   continued rotation on release, drift momentum, re-decision intervals. `MppiTest` pins the scalar
   rollout against the record model, the six keys, tree size, tier ranking and heading diversity.
3. Controller-level: `NavigationTest` / `WaypointPilotTest` on ice. Read the failure dump (tick,
   position, velocity, yaw, angular velocity) before touching a tolerance; prefer adding a
   deterministic case over loosening an assertion.
4. Real ride: `MppiRecorder` writes the last 24 decisions to `config/turneler/mppi-trace.bin`;
   `./gradlew test --tests "*MppiTraceTest*" -Dturnuler.mppiTrace=<path>` replays them on the recorded
   field, re-checks the winning route never clips bare ground, and prints the tick-by-tick gap between
   the model's own rollout and what the boat measured. That gap is the only number that separates
   "the search chose badly" from "the model has stopped describing the boat" — reach for it before
   tuning anything on a bug that only appears on a real map.

Rule of thumb: if a change makes a drift case pass by assuming the boat turns tighter, or by removing
slip from the state, it is not a fix — it is a model that will fail on the next lake.
