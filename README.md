# Icer

**Icer** is the collection repository for a Minecraft 26.2 ice-boat autopilot: the mod implementation,
the portable algorithm engineering knowledge around it, and a browser sandbox for the same controller.

Ice boats have no lateral grip — every turn is a drift. Icer is built around that fact instead of a
"car with a steering angle" model: a local cubic Bezier centre line through ice that supports the whole
hull, plus a measured per-tick W/A/S/D controller that shares one thrust budget between curvature
acceleration and yaw-rate counter-steering. There is no fixed maximum or target speed.

[中文说明 / Chinese README](README_ZH.md)

## What is in this repository

| Path | What it is | What it is for |
| --- | --- | --- |
| [`Minecraft Mod/`](Minecraft%20Mod) | The mod itself — **Turneler**, a client-side Fabric mod for Minecraft 26.2 (Kotlin, JDK 25) | Drive a boat along connected ice automatically and tune it in game |
| [`Agents/skills/icer/`](Agents/skills/icer) | `SKILL.md` — the standalone **Icer algorithm specification** (Chinese) | Re-implement or debug the controller outside this repo, with no Minecraft- or repo-specific dependency |
| [`Agents/skills/Minecraft26.2-IceBoat-Physics-Engineer/`](Agents/skills/Minecraft26.2-IceBoat-Physics-Engineer) | `SKILL.md` — an **engineering protocol for Minecraft 26.2 boat dynamics** (English) | Get the vanilla model right: tick order, damping, control inputs, friction table, slip angle and rotation memory |
| [`Website/`](Website) | `index.html` — a single-file, dependency-free **ice track editor and controller sandbox** ("冰面编辑器") | Draw a track, place a boat and watch the controller solve thrust tick by tick, in a browser |
| `LICENSE`, `.gitignore` | MIT licence and repository ignore rules | — |

## `Minecraft Mod/` — Turneler

A client-side Fabric mod for Minecraft 26.2 (`minecraft_version=26.2`, `loader_version=0.19.5`,
Fabric API `0.159.0+26.2`, mod version `1.0.0`). Press **Right Shift** to open settings, **Esc** to close.

Behaviour, in short:

- One autopilot traces a local centre line through ice supporting the entire 1.375-block hull; there are
  no modes and no external model service.
- A daemon worker scans terrain and fits connected cubic Bezier segments; routes are aligned and
  pointwise-averaged over the last 20 actual ticks before publishing. Fully contributed points become a
  stable prefix; refreshes keep that prefix and replace only the unsteady tail, re-validated against
  full-hull ice and swept collisions.
- The controller reads post-damping measured velocity and yaw rate every tick and applies simultaneous
  W/A/S/D. It picks ice-safe preview points, anticipates bends inside a coasting window, subtracts
  natural ice drag from the braking thrust it needs, and counter-steers residual rotation.
- A green world line shows the route, a white point and amber arrow show the current target, and a
  compact overlay shows status, speed and the actual keys. Route drawing and overlay position are
  configurable; settings persist in `config/turneler.json`.
- Scope is a horizontal diagnostic model: water, vertical motion, entity collisions and server
  corrections are out of scope, and blocked motion is rejected rather than modelled as wall sliding.

Layout and commands:

```text
src/main/kotlin/adreno/turneler/navigation/   platform-free geometry, IcerPathGenerator, IcerController
src/client/kotlin/adreno/turneler/client/     client entrypoint, BoatPilot, MinecraftTerrain, HUD, UI
src/client/java/.../mixin/                    Java client mixins
src/test/kotlin/                              JUnit 5 geometry, controller, tuning, config tests
docs/boat-physics-26.2.md                     source-anchored vanilla physics and model scope
docs/tuning.md                                user-facing tuning reference
AGENTS.md                                     layout, invariants and conventions for contributors
```

```text
./gradlew build        ./gradlew test        ./gradlew runClient
```

Requires JDK 25 or newer; the artifact is `build/libs/turneler-1.0.0.jar`. The navigation package must
stay free of `net.minecraft.*` imports — world access goes through `NavigationEnvironment`. See
[`Minecraft Mod/README.md`](Minecraft%20Mod/README.md) and
[`Minecraft Mod/AGENTS.md`](Minecraft%20Mod/AGENTS.md) for the full description and conventions.

> The CI workflow lives at `Minecraft Mod/.github/workflows/build.yml`. GitHub only reads the repository
> root `.github/workflows/`, so it is stored next to the project as-is and is not triggered automatically.

## `Agents/skills/` — the portable knowledge

- **`icer`** — the algorithm specification as a standalone document: state and coordinate conventions,
  real per-tick dynamics, preview-point selection, thrust direction from curvature and slip, cumulative
  W accounting over intermediate bow angles, bend pre-braking, yaw-rate counter-steering and the WASD
  decision. Explicitly not a trajectory search, MPPI or a car model; coefficients are meant to be
  re-validated on the target track.
- **`Minecraft26.2-IceBoat-Physics_Engineer`** — the engineering protocol for any 26.2 boat dynamics
  model, predictor, controller, planner or simulator, with the complete vanilla boat model (tick order,
  damping, control inputs, friction table) and the slip-angle/rotation-memory maths a naive steering
  model gets wrong.

Both are packaged as agent skills (`SKILL.md` with `name`/`description` front matter), so they can be
dropped into an agent's skill directory and loaded when boat physics work comes up.

## `Website/` — ice track editor

`index.html` is one self-contained file, no build step and no external scripts. Open it in a browser to:

- paint tiles (blue ice, packed ice, normal ice) and place the boat,
- run the controller and step or replay the resulting key sequence (0.25×–8×),
- inspect the working: velocity decomposition, boat/sideways stopping distance, target vs. actual
  thrust, the cumulative-W solver, candidate solutions and search convergence, route projection and
  lateral error, Bezier axis vs. ice sampling, contact block and friction, damping and velocity update,
  key-release glide prediction,
- import and export the track as JSON.

## Licence
MIT for the repository — see [LICENSE](LICENSE). `Minecraft Mod/` keeps the CC0 licence file that came
with the project template.
