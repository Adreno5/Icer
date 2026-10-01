# Minecraft 26.2 Boat Dynamics

Source inspected: Loom's local `minecraft-common-043a8b3edf-26.2-sources.jar`,
`net/minecraft/world/entity/vehicle/boat/AbstractBoat.java` and
`net/minecraft/world/level/block/Blocks.java`.

## Vanilla Tick Order

For a locally controlled boat, `tick()` calls `getStatus()`, `floatBoat()`,
`controlBoat()`, and then `move(MoverType.SELF, deltaMovement)`.
Turneler's mixin runs at the start of `controlBoat`: its measured velocity and
`deltaRotation` have ALREADY received this tick's damping. The first predicted
step must not damp those values a second time.

On land, `floatBoat()` sets `invFriction = landFriction`, then multiplies X/Z
velocity and `deltaRotation` by `invFriction`. The subsequent player-specific
`landFriction /= 2` does not alter the already copied `invFriction` for this tick.
`getStatus()` samples the ground friction again on the next tick.

In `controlBoat()`:

- A subtracts 1 degree/tick from `deltaRotation`; D adds 1. Both cancel.
- Yaw then increases by `deltaRotation`. Releasing keys does not erase rotation.
- W adds 0.04 forward acceleration; S subtracts 0.005.
- A or D alone adds 0.005 forward acceleration, but only with neither W nor S.
- Thrust is added along `(-sin(yaw), cos(yaw))`, using the NEW yaw.
- Existing X/Z momentum is never rotated to the new yaw.

A/D thus provide signed angular acceleration, not steering angle or increasing
angular acceleration (jerk). Ice drift requires a bow/velocity slip angle: W
adds a vector which gradually redirects old momentum. S is much weaker than W.

## Ice Retention

`Blocks.java` assigns `.friction(0.98F)` to regular, packed, and frosted ice;
blue ice uses `.friction(0.989F)`. These are retention multipliers: larger means
less speed and rotation loss. Whether a column carries ice at all is a boolean owned by the route
gate, not a graded perception weight, and neither substitutes for the friction values.

`getGroundFriction()` averages the friction of collision shapes intersecting
the hull-bottom slice from `minY - 0.001` to `minY`, counting each contacted block once
rather than weighting by overlap area. Turneler now uses that slice
and shape intersection rather than a broad height-only contact test. Vanilla
also excludes lily pads; this ice pilot deliberately does not introduce a
block-ID exception for that non-ice case.

## Current behavior: local centre line and tick feedback

`IcerPathGenerator` walks supported ice terrain in the direction of travel and probes the
left and right ice margins to centre each step. It filters the small changes caused by block
edge sampling, then fits cubic Bezier segments through anchors farther apart than the trace
steps. Each fit stays close to the measured centres according to the available ice margin,
and every beam chord and fitted curve is checked with a swept boat collision box. A trace
cut short at three-block steps is retried at 1.5-block steps; narrow bends also fall back
to closer anchors and less filtering. The route is local and has no knowledge of checkpoint order;
forks follow the current heading and the previous line. A worker refreshes the route whenever
its last result arrives, while `IcerController` reads the measured boat state every tick.

Before publishing, `IcerRouteAverager` computes pointwise arithmetic means over a real 20-tick
window (configurable with `history_ticks`, 1 disables it). `BoatPilot` records the raw route in use
on each boat-control tick, including ticks when the worker has not returned a new result, and sends
an immutable history snapshot to the worker. The fresh route occupies the current planning tick;
only the preceding 19 ticks contribute history. Entries expire by tick number, not solve count.
Disabling, dismounting, teleporting, blocked movement and tuning/horizon changes clear the history.

Historical routes are projected onto the fresh route's starting position and sampled at equal
forward arc distances. Routes facing the opposite direction or beyond the existing route-agreement
distance are excluded. Samples past an old horizon's endpoint contribute nothing, rather than being
clamped to the endpoint. Non-ice historical points are excluded. For each point the averager counts
distinct contributing ticks in this window. A full window makes the point stable; connected cubics
between stable points form a frozen prefix. Later worker results retain these points and handles
exactly, splitting only the first remaining cubic to trim travelled geometry. The mutable tail joins
the frozen end tangent and continues toward the fresh horizon; its means still use only raw history.
A lone mature starting point is also retained until passed. If earlier points are missing valid
contributions, mature knots farther ahead remain fixed individually until the stable prefix reaches
them. If no safe new tail can retain those knots, the still-safe old suffix is kept for this refresh.
Missing ticks or repeated solves cannot
make a point mature early, and expired raw samples do not unfreeze existing geometry.

The output interpolates new mean points with connected cubics; shorter handles are tried if necessary.
Both the retained prefix and new tail must pass full-hull ice and swept collision checks. An unsafe
mean tail falls back to fresh tail points, then to just the safe stable prefix if the join still fails.
An invalid stable prefix is discarded; without a stable prefix an unsafe mean uses the fresh safe curve.
Width/friction is measured again from current terrain, including along frozen geometry. History never
contains already averaged curves, which would introduce recursive lag. The average sample interval is
`history_spacing` (default 1 block). Setting `history_ticks` to 1 disables both averaging and freezing.

`MinecraftTerrain.ice(position)` checks every block touched by the 1.375-block hull footprint;
The autopilot uses this full-hull gate. A one-block ice rail or a dry marking under part of the
hull is rejected. Candidate chords and fitted curves sample the hull every quarter block;
shortcut lines sample at intervals no greater than half a block. `clear(from, to)` sweeps the
boat above its ground-contact slice, so low decoration that does not touch the hull can be
passable while a solid obstruction cuts the route. The terrain samples remain discrete checks.

The controller projects onto that line with control-box lower bounds, polyline seeds and a
bounded Newton solve. Icer's arc tables sample each cubic at eight points per control-polygon
block, bounded to 16..4000 samples. Its preview scans tangent changes, then tests the full
hull from the current boat position to the proposed point; seven bisections retreat from an
unsafe shortcut. The current route generator has no painted brush width, so the preview uses
the measured ice width. Target progress stays monotonic on each route.

Icer has no maximum or target speed. A straight requests +0.04 tangential thrust even when
narrow. The bend preview covers the longer of the strong-thrust braking distance and the
natural-drag tail, plus eight ticks of travel. Only corners within their approximate coasting
window contribute required net deceleration. Their lateral budget retains a nonzero narrow-hull
allowance and accounts for curvature changes and reversals. Braking demand is
`B = clamp((0.04 + required - speed*(1-F))/0.08, 0, 1)` when required is positive, otherwise zero.
Thus `a_parallel = 0.04*(1-2B)` subtracts the drag already provided by ice; it does not apply the
same net braking twice. The local corner speed used to estimate coasting distance is not a
cruise target. Current friction and total speed approximate drag; future friction is not forecast.

The controller averages the current and eight future clamped curvatures, then blends preview
tangency before combining unsaturated curvature, position error and sideways velocity. It never
clips current curvature thrust and then adds a potentially opposite preview correction. Feedback
time uses at most three blocks of measured half-width (four with a painted geometric width),
and wide measured ice increases position feedback. Shortcuts probe current and four future
normals only within the target/yaw response range, subtracting inward stopping distance. Near
measured ice edges, lateral thrust has priority; elsewhere requests share one 0.04 thrust circle.
The live generator supplies measured widths, not painted brush widths; the optional painted
route branch retains the specification's preview and coast-braking behavior.

The desired W heading is compared with coasting and weak 0.005 S, but remains the steering
target regardless of which thrust action wins. The route reference yaw rate uses signed along-line
velocity and is bounded to +/-5 degrees/tick. Yaw error requests a rate correction bounded by
response time and a stopping-angle square-root bound; A/D track that rate with a +/-0.8 degree/tick
neutral zone. Both real steering and the cumulative forecast use this same rule. The forecast
adds each intermediate post-turn bow's useful W impulse over up to 28 ticks by default before
assigning pulsed W. A braking conflict reduces W duty; there is no forced W launch or key quota.

S is pulsed by the actual control-tick number, independent of route refreshes: one release
every five ticks, or every ten ticks at speeds up to 0.55. A painted route can suppress S at
low speed with low edge risk, small yaw error and low spin. All these states retain A/D feedback.
An S-release tick with A/D therefore still has vanilla's +0.005 bare-turn thrust; it is not
zero-thrust passive coasting. The missing/blocked-route spin fallback retains A/D plus S.

The client hook runs *after* vanilla drag, so the first forecast step uses measured velocity
and `deltaRotation` without damping again. Subsequent forecast yaw rates damp once per tick.
Float yaw updates add the float angular rate and steering acceleration before adding to yaw,
matching the physical step's rounding order. `IcerPhysics` mirrors float yaw/rate and the Minecraft
sine table; the live controller re-reads the measured boat on the next tick. The preview, binary
safe-distance search and braking request are heuristics, not continuous collision or stopping proofs.
The monotonic target can outlive a shorter new preview on the same route; projection progress
remains the current nearest point, with previous progress used only to resolve equal-distance ties.
If the line is missing, too short, or blocked,
it drops forward thrust and counters any remaining yaw rate with A/D plus S; the HUD names
the scan failure and shows the keys. The route gizmo is drawn from `renderGizmos()`, where the
client renderer calls it, rather than from the input hook. The current preview P is a larger
filled white circle; its amber PV arrow starts at P and follows the route tangent there.
Water, vertical motion, entities and server corrections remain outside this
horizontal model. `IcerTest` covers centre-line fit, a quarter turn, quantized wide ice,
narrow winding lines, strict hull support, preview tangency, yaw counter-steering,
pre-bend braking, intermediate W impulse, float tick order, closed-loop feedback,
stationary starts and invalidation when the ice changes.

`IcerControllerTest` also pins unsaturated spike recovery, coasting windows, natural-drag
compensation, unbounded narrow-straight acceleration, neutral yaw ticks, signed rate bounds,
stable W headings during coasting, S-release timing and continued steering during painted coast
braking. `IcerRegressionTest` checks discrete keys on three tile tracks at both 0.98 and 0.989,
including full-hull and centre support, endpoint progress/distance/plane crossing, maximum/RMS
lateral error, total turn, A/D changes and adjacent counter-turns. It writes `build/icer-regression.csv`.
W/S shares divide by the total number of held keys (W+D counts twice), while duties divide by
executed ticks. Aggregate W share above 60% and S share below 20% are regression targets; full-hull
support and completing each track take priority. Track ice continues past the finish checkpoint:
ending control does not guarantee a stationary boat.

## Configurable heuristics and fixed physics

`IcerSettings` captures bounded, immutable parameter snapshots. `IcerParameter` defines the same defaults,
ranges and descriptions for the runtime, settings UI and JSON configuration. Route generation and tick
feedback both consume these snapshots; changing tuning or horizon cancels the pending result, clears
the old route and re-anchors from measured state. Controller defaults follow the Icer specification;
the spatial fitter's fallback ladder and 20-tick temporal averaging stage are preserved. Existing
values of retained keys survive migration; restoring defaults adopts the new coefficients. Obsolete
narrow-speed braking, yaw-angle deadzone and forced-launch keys are omitted from UI and saved JSON.

User tuning covers scanning, scoring, smoothing, preview, braking, lateral feedback, yaw control and
solver precision. Vanilla 0.04 forward thrust, 0.005 reverse/bare-turn thrust, one-degree yaw acceleration,
friction from the world and the 1.375-block hull remain fixed. Ice checks still require every touched
block to be ice; quarter-block route sampling and at-most-half-block shortcut sampling remain fixed.
No tuning field adds a cruise-speed cap or rotates world momentum with the bow.

See [tuning reference](tuning.md) for all user-visible controls. Removed prediction, waypoint,
DeepLearn, MPPI and Jev runtimes, protocols and external Python helpers are no longer part of the mod.
