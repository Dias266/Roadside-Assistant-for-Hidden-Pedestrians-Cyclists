# 🚗📡 V2X Emergency Braking System — CARLA Simulator

A laboratory exercise implementing an **emergency braking system** that fuses
**Vehicle-to-Everything (V2X)** communication with **radar sensor** data inside
the [CARLA](https://carla.org/) autonomous driving simulator.

---

## Table of Contents

- [Overview](#overview)
- [Prerequisites](#prerequisites)
- [Installation](#installation)
- [Project Structure](#project-structure)
- [Architecture](#architecture)
- [How to Run](#how-to-run)
- [Configuration](#configuration)
- [V2X vs Radar](#v2x-vs-radar)
- [Test Scenarios](#test-scenarios)
- [Known Limitations](#known-limitations)

---

## Overview

Each V2X-equipped vehicle broadcasts a **Basic Safety Message (BSM)** at 10 Hz
containing its position, velocity vector, heading, and brake status. The ego
vehicle receives these messages and computes **Time-to-Collision (TTC)** using
vector arithmetic. A radar sensor runs in parallel as a fallback for
non-equipped obstacles. A **fusion controller** combines both sources using
weighted TTC averaging and applies emergency braking when needed.

### Braking state machine

```
TTC > 5 s          →  NORMAL   (throttle 0.3, brake 0.0)
3 s < TTC ≤ 5 s    →  WARNING  (throttle 0.0, brake 0.3)
TTC ≤ 3 s          →  BRAKING  (throttle 0.0, brake 1.0)  ← emergency
brake_status = True →  WARNING  (cascade pre-warning, regardless of TTC)
```

---

## Prerequisites

| Requirement | Version |
|-------------|---------|
| CARLA Simulator | 0.9.x |
| Python | 3.7+ |
| CARLA Python API | matching CARLA version |
| Jupyter Lab / Notebook | any recent |
| matplotlib *(optional)* | for validation plots |

---

## Installation

### 1. Install CARLA

Download from [carla.org/get-started](https://carla.org/get-started/) and
follow the platform-specific setup guide.

### 2. Add the CARLA Python egg to your environment

```bash
# Option A — permanent (add to shell profile)
export PYTHONPATH=$PYTHONPATH:/path/to/carla/dist/carla-0.9.x-py3.x-linux-x86_64.egg

# Option B — per-notebook (uncomment the first cell in any notebook)
# import sys
# sys.path.insert(0, '/path/to/carla/dist/carla-0.9.x-py3.x-linux-x86_64.egg')
```

### 3. Install Python dependencies

```bash
pip install jupyter matplotlib
```

### 4. Clone / unzip this project

```bash
unzip v2x_emergency_braking.zip
cd v2x_emergency_braking
jupyter lab
```

---

## Project Structure

```
v2x_emergency_braking/
│
├── README.md                              ← This file
│
├── 00_overview.ipynb                      ← Project intro, V2X concepts,
│                                            architecture diagram
│
├── 11_run_simulation.ipynb                ← ▶ Self-contained entry point.
│                                            Run this to start the simulation
│                                            (all classes inlined, no imports
│                                            between notebooks needed)
│
├── config/
│   └── 01_config.ipynb                    ← All shared constants.
│                                            Edit ONLY this file to tune
│                                            thresholds, speeds, and ranges
│
├── sensors/
│   └── 02_radar_sensor.ipynb              ← RadarSensor class.
│                                            Attaches a CARLA radar actor to
│                                            the ego vehicle (fallback source)
│
├── v2x/
│   ├── 03_v2x_message.ipynb               ← BasicSafetyMessage dataclass
│   │                                        (simplified SAE J2735 BSM)
│   ├── 04_v2x_broadcaster.ipynb           ← V2XBroadcaster — background thread
│   │                                        that reads vehicle state and
│   │                                        publishes BSMs to V2XNetwork
│   └── 05_v2x_receiver.ipynb              ← V2XReceiver — filters BSMs by
│                                            range, freshness, and sender ID
│
├── processing/
│   ├── 06_radar_processor.ipynb           ← RadarProcessor — filters raw
│   │                                        detections, computes TTC from
│   │                                        relative radial velocity
│   └── 07_v2x_processor.ipynb             ← V2XProcessor — computes TTC from
│                                            absolute velocity vectors using
│                                            dot-product projection
│
├── control/
│   └── 08_braking_controller.ipynb        ← FusionBrakingController — weighted
│                                            TTC fusion, three-state machine,
│                                            V2X brake-cascade pre-warning
│
├── scenario/
│   └── 09_scenario_manager.ipynb          ← ScenarioManager — spawns vehicles,
│                                            wires all components, runs 10 Hz
│                                            control loop, prints report
│
└── validation/
    └── 10_validation.ipynb                ← Unit tests (no CARLA needed),
                                             TTC timeline plot, test case table
```

---

## Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                        CARLA Simulator                          │
│                                                                 │
│   ┌──────────────┐   BSM @10 Hz    ┌────────────────────────┐  │
│   │ Obstacle Veh │ ────────────── ▶│  V2XNetwork (shared    │  │
│   │ V2XBroad-    │                 │  in-process message bus)│  │
│   │ caster       │                 └───────────┬────────────┘  │
│   └──────────────┘                             │               │
│                                                │ filtered BSMs  │
│   ┌─────────────────────────────── ────────── ▼ ────────────┐  │
│   │                    EGO VEHICLE                           │  │
│   │                                                          │  │
│   │  ┌─────────────┐        ┌──────────────────────────┐    │  │
│   │  │ RadarSensor │        │ V2XReceiver → V2XProcessor│   │  │
│   │  │      ↓      │        │   (vector TTC, cascade)   │   │  │
│   │  │ RadarProc.  │        └────────────┬─────────────┘    │  │
│   │  └──────┬──────┘                     │                  │  │
│   │         │                            │                  │  │
│   │         └──────────────┬─────────────┘                  │  │
│   │                        ▼                                 │  │
│   │            FusionBrakingController                       │  │
│   │         (weighted TTC: 60% V2X, 40% Radar)              │  │
│   │                        ▼                                 │  │
│   │                 VehicleControl                           │  │
│   │            (throttle / brake commands)                   │  │
│   └──────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
```

### V2X TTC formula

```
r  = p_obstacle − p_ego          (relative position vector)
v  = v_obstacle − v_ego          (relative velocity vector)

closing_speed = −(v · r̂)         (projection onto approach direction)

TTC_v2x = |r| / closing_speed
```

### Fusion formula

```
TTC_fused = 0.6 × TTC_v2x + 0.4 × TTC_radar
```

If only one source is available it is used directly (no weighting applied).

---

## How to Run

### Quick start (recommended)

```
1. Start CARLA server
2. Open 11_run_simulation.ipynb in Jupyter
3. Run all cells (Kernel → Run All)
```

### Step-by-step (for learning)

```
1. Start CARLA server
2. Run notebooks in numerical order: 01 → 02 → ... → 09
3. Run 10_validation.ipynb for unit tests and plots (no CARLA needed)
```

### Interrupt safely

Use **Kernel → Interrupt** at any time. The `finally` block in every scenario
notebook always destroys all spawned actors cleanly.

---

## Configuration

All tunable parameters live in `config/01_config.ipynb` (and are also at the
top of `11_run_simulation.ipynb`):

| Constant | Default | Description |
|----------|---------|-------------|
| `V2X_BROADCAST_RANGE` | 300 m | Simulated radio range |
| `V2X_BSM_RATE_HZ` | 10 Hz | BSM transmission frequency |
| `V2X_MESSAGE_TTL` | 0.5 s | Discard BSMs older than this |
| `RADAR_RANGE` | 50 m | Radar max detection range |
| `TTC_THRESHOLD_BRAKE` | 3.0 s | Full emergency brake below this |
| `TTC_THRESHOLD_WARN` | 5.0 s | Gentle brake / warning below this |
| `FUSION_WEIGHT_V2X` | 0.6 | Weight for V2X TTC in fusion |
| `FUSION_WEIGHT_RADAR` | 0.4 | Weight for radar TTC in fusion |
| `EGO_TARGET_SPEED` | 60 km/h | Ego vehicle cruise speed |
| `SIMULATION_DURATION` | 30 s | Scenario run time |
| `EGO_SPAWN_INDEX` | 0 | Spawn point for ego vehicle |
| `OBSTACLE_SPAWN_INDEX` | 10 | Spawn point for obstacle vehicle |

---

## V2X vs Radar

| Capability | Radar only | V2X + Radar fusion |
|------------|-----------|---------------------|
| Detection range | ~50 m | ~300 m |
| Non-line-of-sight | ❌ | ✅ |
| Works in fog / rain | ✅ | ✅ |
| Absolute velocity known | ❌ (relative only) | ✅ |
| Brake cascade warning | ❌ | ✅ |
| Requires equipped vehicles | ❌ | ✅ |
| Intersection / angle approach | Limited | ✅ (vector TTC) |

---

## Test Scenarios

| # | Setup | Expected result |
|---|-------|----------------|
| A | Ego 60 km/h → static obstacle 40 m ahead, V2X only | ✅ Emergency brake activated |
| B | Ego 60 km/h → obstacle moving away faster | ✅ No braking triggered |
| C | Obstacle transmits `brake_status = True` | ✅ Cascade pre-warning before TTC threshold |
| D | V2X disabled, radar only | ✅ Radar fallback activates braking |
| E | Both V2X + radar active, conflicting TTC readings | ✅ Weighted fusion picks conservative result |
| F | Obstacle > 50 m away (beyond radar range) | ✅ V2X detects early, radar misses |

To run scenario F: increase `OBSTACLE_SPAWN_INDEX` distance or set
`EGO_TARGET_SPEED = 80`.

---

## Known Limitations

- **Simulated network only** — the V2XNetwork is an in-process Python object.
  A real deployment uses DSRC 802.11p (5.9 GHz) or C-V2X (PC5 / Uu).
- **Constant-velocity TTC** — the formula assumes no acceleration. A
  kinematic TTC model would be more accurate for braking obstacles.
- **No map matching** — spawn indices must be on the same road segment for
  the scenario to work. Verify in the CARLA spectator before running.
- **Single obstacle** — the broadcaster is attached to one vehicle. Adding
  more broadcasters (one per vehicle) scales the system to traffic.
- **No ASN.1 encoding** — BSMs are plain Python dataclasses. Real BSMs are
  UPER-encoded binary packets.

---

## References

- [CARLA Simulator documentation](https://carla.readthedocs.io/)
- [SAE J2735 — DSRC Message Set Dictionary](https://www.sae.org/standards/content/j2735_202309/)
- [ETSI EN 302 637-2 — CAM (Cooperative Awareness Message)](https://www.etsi.org/deliver/etsi_en/302600_302699/30263702/)
- [IEEE 802.11p — WAVE standard](https://standards.ieee.org/ieee/802.11p/4133/)
