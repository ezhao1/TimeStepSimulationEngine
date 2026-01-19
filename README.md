# Deterministic Fixed-Timestep Simulation Engine (C++)

Small, focused C++ simulation engine built to explore **deterministic physics simulation**, **fixed-timestep integration**, and **force-based motion modeling**.

The goal is not rendering or gameplay, but **correctness, determinism, and clarity of simulation structure**, similar to what is used in real-time engines, vehicle simulators, and networked games.

---

## Goals

- Deterministic simulation independent of frame rate
- Fixed-timestep physics with accumulator
- Stable numerical integration
- Clear separation between:
  - Simulation state
  - Forces / models
  - Integration logic
- Engine-style architecture suitable for extension (vehicles, impulses, collisions)

---

## Key Concepts Implemented

### Fixed Timestep with Accumulator

The simulation advances using a fixed `dt` regardless of frame rate:

```text
frame_dt → accumulator → N fixed updates → interpolation
