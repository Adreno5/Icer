# AGENTS.md

Turneler is a client-side Fabric mod for Minecraft 26.2. It has one ice-boat autopilot:
a local cubic Bezier centre-line generator and measured per-tick drift controller.

## Layout

Loom uses `splitEnvironmentSourceSets()`.

- `src/main/kotlin/adreno/turneler/navigation/`: platform-free geometry, state/input types,
  `IcerPathGenerator`, `IcerController`, `IcerPhysics`, `IcerSettings` and its parameter registry.
- `src/main/kotlin/adreno/turneler/Turneler.kt`: mod initializer and identifier helper.
- `src/client/kotlin/adreno/turneler/client/`: client entrypoint, `BoatPilot`, `MinecraftTerrain`,
  config, native settings widgets and HUD.
- `src/client/java/adreno/turneler/client/mixin/`: Java client mixins, registered in
  `src/client/resources/turneler.client.mixins.json`.
- `src/test/kotlin/`: JUnit 5 geometry, controller, tuning and configuration migration tests.
- `docs/boat-physics-26.2.md`: source-anchored vanilla physics and model scope.
- `docs/tuning.md`: user-facing tuning reference.
- `run/`: ignored development state; `run/config/turneler.json` holds dev settings.
- `build/`: ignored output; `build/libs/turneler-<version>.jar` is the mod artifact.

## Hard invariant: navigation stays platform-free

Nothing under `adreno.turneler.navigation` may import `net.minecraft.*` or `net.fabricmc.*`.
World access goes through `NavigationEnvironment`: boolean `iceBlock` and full-hull `ice`, swept
`clear`, and block `friction`. Implement world/entity queries client-side in `MinecraftTerrain`.

## Commands

Use the Gradle wrapper from the repository root (Gradle 9.5.1, JDK 25 or newer):

```text
./gradlew build
./gradlew test
./gradlew test --tests "*IcerTest*"
./gradlew runClient
./gradlew runServer
./gradlew runDatagen
./gradlew genSources
```

Add `--offline` when the dependency cache is warm and network access is unavailable.
Never commit `run/` or `build/`.

## Runtime and settings

`BoatPilot` refreshes local routes on one daemon planner worker, using a captured hull geometry and
immutable `IcerSettings`. `IcerRouteHistory` records raw routes by actual boat-control tick, and
`IcerRouteAverager` aligns and averages their points before publishing. Never feed averaged output
back into history; validate mean curves against full-hull ice and collisions and resample their
width/friction. The live controller consumes post-damping measured state every tick.
Changing parameters or horizon discards pending results and re-anchors. There are no other modes,
checkpoint integrations, inference bridges, Python services or MPPI recorders.

The settings UI uses iOS-style grouped surfaces and progressive disclosure. Show Turneler as the
product name; do not emphasize algorithm names in UI. Esc exits; saving is silent. Avoid redundant
exit buttons or implementation/status copy about applying and saving settings. `SettingsViewState`
persists the last page/category and independent scroll offsets. Do not reset navigation history when
restoring algorithm defaults; clamp restored offsets to the current viewport.

All tunable heuristic values belong in `IcerParameter`, including defaults, bounds, steps and
explanations. The UI and JSON expose the same registry. `IcerSettings` sanitizes non-finite values,
normalizes integer values and enforces related limits. Preserve defaults when exposing parameters.
Keep vanilla physics, hull dimensions, numerical tolerances and conservative safety sampling fixed.

## Conventions

- Kotlin except Java mixins. Prefix injection handlers with `turneler$`.
- Match indentation in edited files; do not reformat unrelated code.
- Read dependency versions from `gradle.properties`.
- Register mixins in their environment-specific JSON; satisfy `overwrites.requireAnnotations`.
- Client-only code stays in `src/client/`; common code must not reference client classes.
- Log through SLF4J, never `println`.
- New config fields need defaults and sanitization; user tuning also needs a UI control.
- Keep README current behavior, physics notes and tuning documentation synchronized.

## Physics and verification

The authority is `docs/boat-physics-26.2.md`.

- `controlBoat` HEAD runs after vanilla damping. Do not damp measured velocity or yaw rate twice.
- A/D are angular acceleration; yaw changes before propulsion. Old world momentum never rotates
  with the bow. Forward thrust redirects drift, so simultaneous W+A and W+D must remain possible.
- Released steering leaves residual rotation and needs counter-steering.
- Ice is boolean for route support; every block touched by the 1.375-block hull must carry ice.
  Friction remains a separate dynamics input: 0.98 ice and 0.989 blue ice.
- Reject collisions; do not model wall sliding. Water, vertical movement, entities and server
  corrections are outside the horizontal diagnostic model. Live control re-anchors every tick.
- No fixed maximum or target speed. Bend and narrow-ice heuristics request a thrust direction.

JUnit 5 tests use anonymous environments without starting Minecraft. `IcerTest` pins routes,
full-hull support, preview tangency, intermediate W impulse, yaw inertia, stationary starts,
closed-loop feedback, tuning effects and all key combinations on both ice frictions.
`ChainPathTest` pins cubic chain geometry. `TurnelerConfigTest` pins legacy config migration,
round trips and malformed parameter recovery. Read failure dumps before changing tolerances;
prefer deterministic regression cases. `./gradlew build` includes tests and is the CI artifact gate.

Current verification after the Icer controller/configuration alignment: 59 tests, no failures or skips.
Removed modes' historical baseline failures are no longer part of the retained test suite.
