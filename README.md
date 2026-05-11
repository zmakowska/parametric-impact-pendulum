Dynamics of a Parametrically Excited Pendulum with Finite-Range Impacts
======================================================================

Numerical simulation | experimental validation | nonlinear dynamics | impact systems

Description:
------------
This repository presents my research project on the dynamics of the clackers toy:
two balls connected by a string with a vertically driven midpoint.

Between impacts, the system behaves like a parametrically driven pendulum. During impacts,
finite-size collision geometry introduces non-smooth dynamics. The project investigates where
the classical Mathieu-equation description fails and how finite collision radius changes the
long-term behavior of the system.

The main result is the identification of a third dynamical regime: stable oscillations with
finite, non-zero amplitude. This regime is not predicted by the classical zero-radius Mathieu
model, which only predicts damped oscillations or parametric resonance with growing amplitude.

Authors:
-------
Zofia Makowska  
Łukasz Gładczuk  
Science Club Fenix


Key Result:
-----------
The classical Mathieu model predicts two main regimes:

1. Damped oscillations converging to rest.
2. Parametric resonance with growing amplitude.

In this project, we identified a third regime caused by finite-radius impacts:

    stable oscillations with finite, non-zero amplitude.

This regime appears because real balls collide at a finite distance from the axis of symmetry.
As a result, the collision geometry changes the phase-space structure of the system. The
finite-radius model reproduces experimentally observed behavior across multiple system
parameters, including ball radius, string length, driving amplitude and material-dependent
restitution.


What we Built:
-------------
- A C++ simulation of a parametrically driven impact pendulum.
- Collision handling for finite-radius impacts.
- Parameter sweeps over dimensionless driving frequency and amplitude.
- RMS amplitude maps used to classify long-term behavior.
- An experimental setup with controlled driving frequency and amplitude.
- A video-analysis pipeline to extract angular position from recordings.
- Quantitative comparison between numerical predictions and experimental measurements.


Scientific Motivation:
----------------------
Parametric resonance appears in many physical systems, from mechanical instabilities to
precision devices such as ion traps and mass spectrometers. The standard reference model is
the vertically driven pendulum described by the Mathieu equation. However, real systems often
include impacts, constraints, finite-size effects and energy losses.

The Lato-Lato toy provides a simple experimental system for studying how realistic collision
geometry modifies parametric excitation.


Model Overview:
---------------
Between impacts, the angular motion is modeled as a parametrically driven pendulum:

```math
\ddot{\alpha} + 2\beta \dot{\alpha}
+ \left(\frac{1}{\delta^2} + \epsilon \sin \tau\right)\sin\alpha = 0
```

where:

- $\alpha$ is angular displacement,
- $\delta = \omega / \omega_0$ is dimensionless driving frequency,
- $\epsilon = H / L$ is dimensionless driving amplitude,
- $\beta$ is damping.

Finite ball radius introduces impacts at:

```math
\alpha_c = \sin^{-1}\left(\frac{R}{R+L}\right)
```

where:

- $R$ is ball radius,
- $L$ is string length.

The collision model includes oblique collision geometry and velocity-dependent restitution
based on contact mechanics.


Numerical Simulation:
---------------------
The simulation performs long-time integration and computes RMS amplitude after transients:

```math
RMS = \sqrt{\frac{1}{T}\int_{t_0}^{t_0+T}\alpha(t)^2\,dt}
```

The $RMS$ value is used to classify the long-term response of the system across parameter space.


Simulation Features:
--------------------
- Adaptive Runge-Kutta integration.
- Collision detection and event handling.
- Parameter sweeps over delta and epsilon.
- RMS-based classification of oscillation regimes.
- CSV output for plotting and analysis.
- Parallel execution for faster parameter-map generation.
- Configurable parameters read from a text configuration file.


Experimental Setup:
-------------------
The experimental setup allowed control of:

- driving frequency,
- driving amplitude,
- string length,
- ball radius,
- ball material and coefficient of restitution.

Oscillations were recorded after a long equilibration period and analyzed from video to extract
angular position as a function of time. The experimental amplitude was measured using the same
RMS-based approach as in the numerical simulation.


Results:
--------
The finite-radius model reproduces experimentally observed behavior across multiple system
parameters, including:

- ball radius,
- string length,
- driving amplitude,
- material-dependent restitution.

The most important finding is that finite-radius impacts create stable finite-amplitude
oscillations. This behavior disappears in the idealized zero-radius model.

The project shows that a small physical detail — the finite radius of the colliding balls —
can qualitatively change the long-term dynamics of a parametrically excited system.


Repository Structure:
---------------------
    src/        C++ simulation code and example configuration
    data/       sample CSV outputs
    figures/    selected figures from simulation and experiment
    docs/       abstract, poster and project write-up


Requirements:
-------------
- g++ with C++11 support or later
- POSIX threads, linked via -pthread
- Linux/macOS terminal environment recommended
- Optional: taskset on Linux for binding execution to selected CPU cores


Compilation:
------------
To compile the program, run:

    g++ -std=c++11 -O3 clackers.cpp -o clackers -pthread

This command compiles the source file "latolato.cpp" with optimization level 3 and links
with pthreads.


Running the Program:
--------------------
The executable expects two command-line arguments: a configuration file and a result file.

Basic run:

    ./clackers config_example.txt result.csv

On Linux, to run the simulation on specific CPU cores, for example cores 4 to 15:

    time taskset -c 4-15 ./clackers config_example.txt result.csv

Explanation:

- "config_example.txt" is the configuration file containing simulation parameters.
- "result.csv" is the output CSV file where simulation results will be saved.
- "taskset -c 4-15" binds the process to CPU cores 4 through 15.
- "time" reports total execution time.


Configuration File Format:
--------------------------
The configuration file should contain key-value pairs, one per line. Example:

    # Simulation parameters
    t_end = 5000.0
    tol = 1e-6
    initial_h = 0.1
    h_min = 1e-10
    max_val = 1e8
    rms_start = 4500.0

    # Collision parameters
    collision_a0 = 0.1
    cor = 0.9
    min_rebound = 1e-6

    # Grid parameters
    num_delta = 100
    num_epsilon = 100
    delta_min = 0.5
    delta_max = 4.0
    epsilon_min = 0.0
    epsilon_max = 0.5

    # Initial conditions
    initial_alpha = 1.0
    initial_alphaprime = 0.0

    # Single simulation parameters
    delta = 1.0
    epsilon = 0.1

    # Parallelism
    num_threads = 8

Adjust these values depending on the desired simulation range and resolution.


Output:
-------
The program writes a CSV file with the following columns:

    delta,epsilon,final_alpha,final_alpha_prime,steps,rms

Each row corresponds to a simulation run for a specific pair of delta and epsilon values.
The RMS column is used to classify the long-term response of the system.


Materials:
----------
- docs/abstract.pdf — project abstract
- docs/poster.pdf — research poster
- docs/project_writeup.pdf — extended project write-up
- figures/ — selected plots and experimental setup images
- data/ — sample simulation outputs


What I Learned:
---------------
This project taught me how to move between:

- physical intuition,
- analytical modeling,
- numerical simulation,
- experimental design,
- measurement and video analysis,
- debugging disagreement between model and reality.

The most important lesson was that realistic geometry can matter qualitatively. In this system,
the finite radius of the colliding balls is not a small correction; it creates a new stable
dynamical regime absent from the idealized Mathieu model.


License:
--------
No license is currently provided. All rights are reserved by the author.


Contact:
--------
For questions about this project, please contact:

Zofia Makowska  
GitHub: zmakowska
