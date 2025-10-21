"""

Ball-on-a-Hill PID Control Simulation
-------------------------------------

Simulates a ball rolling on a parabolic hill (y = x²) while a PID controller
automatically adjusts the hill tilt to move the ball to a target position.

Concepts demonstrated:
- System dynamics (position, velocity, acceleration)
- Feedback control (PID)
- Real-time animation and performance metrics

Written by Krina Amin

"""


import matplotlib.pyplot as plt
import numpy as np


def simulate_ball(Kp, Ki, Kd, dt=0.1, total_time=10.0):
    """
    Runs a PID control simulation of a ball rolling on a parabolic hill.
    
    Inputs:
        Kp, Ki, Kd (float): PID controller gains.
        dt (float): simulation time step.
        total_time (float): total simulation duration (s).
    
    Outputs:
        times, positions, targets, errors (np.ndarray)
    """

    # Defining initial variables and lists
    x = 0.5
    v = 0
    time = 0.0
    positions = [] # Initially empty
    targets = []  # Initially empty
    times = [] # Initially empty

    # Initial integral accumulation
    integral_error = 0

    # goal x positon
    x_target = 0.3 

    # Animation logistics
    plt.ion()  # interactive mode on
    fig, ax = plt.subplots()
    point, = ax.plot([], [], 'ro', label='Ball')  # empty red point
    target_dot, = ax.plot([], [], 'go', label='Target')  # empty green target marker

    # Creating parabolic curve
    x_vals = np.linspace(-1, 1, 200)
    h_vals = x_vals**2

    # Plotting data
    ax.plot(x_vals, h_vals, 'b-', label='Hill Surface')

    # Setting graph labels
    ax.set_xlim(-0.6, 0.6)
    ax.set_ylim(0, 0.4)
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    plt.title("PID-controlled ball tracking a moving target")

    steps = int(total_time / dt)

    # Loop updating variables and lists as time goes on
    for step in range(steps):
        time += dt

        # Moving target step change (after 5 seconds)
        if time < 5.0:
            x_target = 0.0
        else:
            x_target = 0.3
    
        # PID control
        error = x_target - x  # measures how far the ball is from the goal
        d_error = -v  # derivitive of position error
        integral_error += error * dt # keeps track of the running sum of error
        u = Kp * error + Kd * d_error + Ki * integral_error # computes the control push

        # Dynamics
        a = -5 * x - 0.5 * v + u  # combines dynamics with control force
        v = v + a * dt
        x = x + v * dt
    
        # Updates lists
        times.append(time)
        positions.append(x)
        targets.append(x_target)

        # Updates live plot
        point.set_data([x], [x**2]) # Moves red dot according to data
        target_dot.set_data([x_target], [x_target**2]) # Moves green dot according to data
        plt.pause(0.02)

    plt.ioff()
    plt.close(fig)

    # Convert to arrays
    return np.array(times), np.array(positions), np.array(targets), np.array(targets) - np.array(positions)


def compute_metrics(times, positions, targets, errors, dt):
    """
    Computes control performance metrics.
    """

    # Find the step start
    step_start = np.where(targets != targets[0])[0][0]
    t0 = times[step_start]
    target_value = targets[step_start]

    # Rise time 
    try:
        above_10 = np.where(positions >= 0.1 * target_value)[0][0]
        above_90 = np.where(positions >= 0.9 * target_value)[0][0]
        rise_time = times[above_90] - times[above_10]
    except IndexError:
        rise_time = None

    # Overshoot percentage
    overshoot = (np.max(positions) - target_value) / target_value * 100 if target_value != 0 else 0

    # Settling time
    within_band = np.abs(errors) <= 0.05 * abs(target_value)
    settling_idx = np.argmax(np.logical_and(within_band, np.arange(len(times)) > above_90))
    settling_time = times[settling_idx] - t0 if settling_idx > 0 else None

    # Steady-state error
    samples_last_1s = int(1 / dt)
    steady_state_error = np.mean(errors[-samples_last_1s:])

    return rise_time, overshoot, settling_time, steady_state_error

def plot_results(times, positions, targets, errors, metrics):
    """
    Plots position tracking and error curves, and prints metrics.
    """
    # Plot position vs target over time
    plt.figure()
    plt.plot(times, targets, 'g--', label='Target Position')
    plt.plot(times, positions, 'r-', label='Ball Position')
    plt.xlabel("Time (s)")
    plt.ylabel("Horizontal Position (m)")
    plt.title("PID Tracking Performance Over Time")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Error plot
    plt.figure()
    plt.plot(times, errors, 'k-')
    plt.xlabel("Time (s)")
    plt.ylabel("Tracking Error (m)")
    plt.title("Tracking Error Over Time")
    plt.grid(True)
    plt.tight_layout()
    plt.show()

    # Metrics output
    print("\n--- PID PERFORMANCE METRICS ---")
    print(f"Rise time:        {rise_time:.2f} s" if rise_time else "Rise time:        Not reached 90% of target.")
    print(f"Overshoot:        {overshoot:.1f} %")
    print(f"Settling time:    {settling_time:.2f} s" if settling_time else "Settling time:    Not yet settled within ±5% band.")
    print(f"Steady-state err: {steady_state_error:.4f} m")


def main():
    """
    Entry point: get user input, run simulation, and display results.
    """
    print("PID Ball-on-a-Hill Simulation")
    try:
        Kp = float(input("Enter Kp: "))
        Ki = float(input("Enter Ki: "))
        Kd = float(input("Enter Kd: "))
    except ValueError:
        print("Invalid input. Using default values.")
        Kp, Ki, Kd = 10.0, 2.0, 5.0

    dt = 0.1
    total_time = 12.0

    times, positions, targets, errors = simulate_ball(Kp, Ki, Kd, dt, total_time)
    metrics = compute_metrics(times, positions, targets, errors, dt)
    plot_results(times, positions, targets, errors, metrics)


if __name__ == "__main__":
    main()
