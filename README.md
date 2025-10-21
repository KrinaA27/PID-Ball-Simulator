# Ball-on-a-Hill PID Control Simulation

**Author:** Krina Amin  
**Language:** Python  
**Category:** Systems & Control Simulation

---

## Overview

This project simulates a ball rolling on a parabolic hill (y = x²) that is automatically balanced by a PID controller.  
The goal is to keep the ball at a desired horizontal position (`x_target`) by adjusting the hill’s "tilt" (the control input `u`).

This simulation introduces the foundation of control engineering — the concept of feedback.

By experimenting with Kp, Ki, and Kd, you can observe:
- How proportional control affects responsiveness.
- How integral control removes steady-state error.
- How derivative control dampens oscillations.

It’s a simple, visual, and beginner-friendly way to learn how PID controllers (used in robotics, automation, and embedded systems) stabilize dynamic systems.

---

## Concepts Demonstrated
- **System Dynamics:** Position, velocity, and acceleration modeled via physics equations.  
- **Feedback Control (PID):**
  - **Proportional (P):** Reacts to current error.
  - **Integral (I):** Accounts for accumulated past error.
  - **Derivative (D):** Predicts future error rate.  
- **Real-Time Simulation:** Updates motion using small time steps (`dt`) and shows results in real-time animation.  
- **Performance Metrics:** Automatically calculates rise time, overshoot, settling time, and steady-state error.

---

## How It Works

### System Model
The ball’s horizontal motion is approximated by:
\[
a = -5x - 0.5v + u
\]
where:
- \(x\) = position  
- \(v\) = velocity  
- \(a\) = acceleration  
- \(u\) = control force (hill tilt angle, computed by PID)

### Control Law
\[
u = K_p e + K_i \int e\,dt + K_d \frac{de}{dt}
\]
where \(e = x_{target} - x\) is the error between the desired and current position.

---

## Features

- Real-time animated simulation of the ball tracking the moving target.
- Customizable PID gains (`Kp`, `Ki`, `Kd`).
- Automatic performance metric calculations:
  - Rise time  
  - Overshoot (%)  
  - Settling time  
  - Steady-state error
- Two clear plots:
  1. Position tracking over time  
  2. Tracking error vs. time

---

### Prerequisites
Make sure you have Python 3 installed with these libraries:
pip install matplotlib numpy

---

### How to Run
1. Download the file and run it through the Python IDLE, or enter python ball_pid_sim.py into the command prompt.
2. When prompted, enter PID gain values.
3. Code will run the simulation.

---

### References
Modern Control Systems — Dorf & Bishop
Feedback Systems: An Introduction for Scientists and Engineers — Åström & Murray
[Wikipedia: PID Controller](http://taggedwiki.zubiaga.org/new_content/25fb23f948080b5e58f50011fae831d6)
