# Turneler

A client-side Fabric mod for Minecraft 26.2 that drives boats along connected ice.
Press **Right Shift** to open settings and **Esc** to close them.

## Current behavior

- One autopilot traces a local centre line through ice that supports the entire boat hull. There is no mode selector or external model service.
- A daemon worker scans the terrain and fits connected cubic Bezier segments. Before publishing, it aligns forward arc distances and takes pointwise arithmetic means over the last 20 actual ticks of raw routes. Held routes count once per tick; already averaged output is never added to history. The controller reads measured, post-damping velocity and yaw rate every tick, then applies simultaneous W/A/S/D input.
- Points that receive a full window of contributing ticks become fixed. Later refreshes retain the stable prefix and its cubic handles, trim travelled geometry and replace only the unsteady tail. Old horizons stop contributing beyond their endpoints, so averaging does not pull the new horizon backwards. The retained prefix and new tail are checked against full-hull ice and swept collisions; unsafe mean tails use fresh points or retain just the safe prefix. An invalid stable prefix is discarded. Width and friction are sampled again on the output curve.
- The controller chooses ice-safe preview points, anticipates bends within a coasting window and subtracts natural ice drag from the required braking thrust. Narrow straights keep full W. Unsaturated curvature and drift feedback share the physical thrust budget, and bounded yaw-rate feedback counter-steers rotation. There is no fixed maximum or target speed.
- The best W heading remains the steering target during coasting and S pulses. Useful thrust at every intermediate bow angle contributes to W duty. S releases at least once every 5 ticks, or every 10 ticks below 0.55 blocks/tick; A/D feedback continues on those release ticks, including vanilla's 0.005 bare-turn thrust.
- Missing, short or blocked routes release forward thrust and counter residual spin. Changes to tuning discard the pending route and refresh from measured state.
- The green world line shows the route; a white preview point and amber tangent arrow show the current target. Route visibility and drawing through terrain are configurable.
- A compact status overlay shows operating status, speed and actual keys. It can be positioned with sliders, by dragging, or with arrow keys in the placement view.
- Settings persist in `config/turneler.json`. Legacy mode fields are ignored; existing enabled, route horizon and display settings survive migration. Obsolete fields disappear when settings are saved.
- Water, vertical motion, entity collisions and server corrections are outside the horizontal diagnostic model. Route checks reject blocked motion instead of simulating wall sliding.

## Settings

The native Minecraft screen uses iOS-style grouped surfaces, segmented navigation, animated switches, restrained blue controls and a blurred backdrop. Esc closes settings; changes and saving happen quietly. The last page, tuning category and each page's scroll position are remembered across reopening and game restarts.

- **Driving**: autopilot and route horizon (24–72 blocks).
- **Tuning**: route generation, route scoring, curve fitting, safe preview, drift feedback, bend braking, steering, and launch/solver settings. Each parameter has a bounded range, a tooltip and a default. Restore defaults applies to the current category, or to all tuning on the category index.
- **Display**: route preview, drawing through terrain, status overlay visibility and position. Esc exits the placement view back to settings.

The tuning registry is `IcerParameter` in `navigation/IcerSettings.kt`; the JSON `parameters` object exposes the same keys as the UI. Controller defaults follow the Icer specification, including feedback-width bounds, drag-aware bend braking, yaw-rate response and reverse pulses. Existing values of retained parameters survive; restoring tuning defaults adopts the new coefficients. Removed narrow-speed braking, angle-switch and forced-launch parameters are ignored and disappear on save. Spatial fitting and the default 20-tick averaging window are preserved. Set `history_ticks` to 1 to disable temporal averaging. The averaging sample distance is `history_spacing` (default 1 block). Settings are captured as immutable snapshots for the planner and applied to measured tick feedback. Invalid or non-finite values revert to defaults; integer controls and related limits are normalized. Vanilla thrust, angular acceleration, hull dimensions, numerical tolerances and conservative full-hull safety sampling are fixed rather than user tuning.

## Development

Requires JDK 25 or newer. Use the Gradle wrapper from the repository root:

```text
./gradlew build
./gradlew test
./gradlew runClient
```

Add `--offline` when dependencies are cached and network access is unavailable. The mod jar is `build/libs/turneler-1.0.0.jar`; development state in `run/` is ignored by Git.

The navigation package stays platform-free. Minecraft world queries belong in `MinecraftTerrain` and pass through `NavigationEnvironment`. Tests cover route geometry, full-hull ice support, closed-loop driving, tick order and input combinations, temporal jitter reduction, tick-window expiry, stable-prefix retention and tail updates, safe averaging, tuning snapshots, and legacy configuration migration. `IcerRegressionTest` replays discrete keys on narrow straight, quarter-turn and alternating-bend tile tracks at both ice frictions, reporting completion, full-hull support, lateral error, key shares/duties and counter-steering in `build/icer-regression.csv`. These horizontal diagnostic results do not establish safety on every Minecraft track.

See [boat physics notes](docs/boat-physics-26.2.md) and [tuning reference](docs/tuning.md).
