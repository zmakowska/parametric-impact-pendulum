# Dynamics of a Parametrically Excited Pendulum with Finite-Range Impacts

**Numerical simulation · experimental validation · nonlinear dynamics · impact systems**

This repository presents my research project on the dynamics of the clackers toy: two balls connected by a string with a vertically driven midpoint. Between impacts, the system behaves like a parametrically driven pendulum; during impacts, finite-size collision geometry introduces non-smooth dynamics.

The project studies where the classical Mathieu-equation description fails and how finite collision radius changes the long-term behavior of the system.

---

## Key result

The classical Mathieu model predicts two main regimes:

1. damped oscillations converging to rest,
2. parametric resonance with growing amplitude.

In this project, I identified a third regime caused by finite-radius impacts:

**stable oscillations with finite, non-zero amplitude.**

This regime is not predicted by the classical zero-radius Mathieu model. The key mechanism is that real balls collide at a finite distance from the axis of symmetry, so the collision geometry changes the system’s phase-space structure.

---

## What I built

- A C++ simulation of a parametrically driven impact pendulum.
- Collision handling for finite-radius impacts.
- Parameter sweeps over dimensionless driving frequency and amplitude.
- RMS amplitude maps used to classify long-term behavior.
- An experimental setup with controlled driving frequency and amplitude.
- A video-analysis pipeline to extract angular position from recordings.
- Quantitative comparison between numerical predictions and experimental measurements.

---

## Why this matters

Parametric resonance appears in many physical systems, from mechanical instabilities to precision devices such as ion traps and mass spectrometers. The standard reference model is the vertically driven pendulum described by the Mathieu equation. However, real systems often include impacts, constraints, finite-size effects and energy losses.

The clackers toy provides a simple experimental system for studying how realistic collision geometry modifies parametric excitation.

---

## Model overview

Between impacts, the angular motion is modeled as a parametrically driven pendulum:

```math
\ddot{\alpha} + 2\beta \dot{\alpha}
+ \left(\frac{1}{\delta^2} + \epsilon \sin \tau\right)\sin\alpha = 0
