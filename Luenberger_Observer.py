import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt
from ackermann import ackermann_pole_placement as apc

# Declare Variables
K_m = 1.695  # motor gain constant
Tau_m = 0.024  # motor time constant

# Define system matrices
A = np.array([[0, 1],
              [0, -(1/Tau_m)]])  # System dynamics
B = np.array([[0],
              [K_m/Tau_m]])       # Input matrix
C = np.array([[1, 0]])    # Output matrix

# Feedback control gain (chosen for stability)
desired_pole = np.array([-2,-3])
K = apc(A,B,desired_pole)
# K = [5,2]

# Luenberger observer gain 
L = np.array([[240],  
              [19200]])  

# Scaling term
N_inv = -C @ np.linalg.inv(A - B @ K) @ B
N = np.linalg.inv(N_inv)

# Simulation parameters
dt = 0.001  # Reduced time step for more accurate simulation
t_final = 10  # Final time
t = np.arange(0, t_final, dt)  # Time vector

# Initialize state and observer state
x = np.array([[0],
              [1]])  # Initial true state
x_hat = np.array([[0],
                  [0]])  # Initial estimated state (observer)

error = np.array(x-x_hat)

# Frequency of the sine wave input (in Hz)
f = 0.5  # 0.5 Hz (for example)
omega = 2 * np.pi * f  # Angular frequency

# Initialize arrays to store the results
x_history = []
x_hat_history = []
error_history = []
u_history = []
r_history = []  # To store the reference signal (sine wave)

# Function to compute system dynamics dx/dt
def system_dynamics(x, u):
    return A @ x + B @ u

# Function to compute observer dynamics dx_hat/dt
def observer_dynamics(x_hat, u, y):
    return A @ x_hat + B @ u + L @ (y - C @ x_hat)

def error_dynamics(x,x_hat):
    return x- x_hat

# Runge-Kutta 4th order (RK4) method
def rk4_step(f, x, u, dt, *args):
    k1 = f(x, u, *args)
    k2 = f(x + 0.5 * dt * k1, u, *args)
    k3 = f(x + 0.5 * dt * k2, u, *args)
    k4 = f(x + dt * k3, u, *args)
    return x + (dt / 6) * (k1 + 2 * k2 + 2 * k3 + k4)

# Simulate the system and observer over time
for i in range(len(t)):
    # Step input: 0 for t < 2s, 1 for t >= 2s
    if t[i] < 0:
        r = np.array([[0]])  # Step input remains 0
    else:
        r = np.array([[1]])  # Step input changes to 1 after 2 seconds

    # Sine wave reference signal
    # r = np.array([[np.sin(omega * t[i])]])  # Reference as a sine wave

    # Control law (using estimated state)
    u = -K @ x_hat + N @ r

    # Observer dynamics (state estimation)
    y = C @ x  # Measurement (output)

    # Apply RK4 for both system and observer dynamics
    x = rk4_step(system_dynamics, x, u, dt)
    x_hat = rk4_step(observer_dynamics, x_hat, u, dt, y)
    error = rk4_step(error_dynamics,x,x_hat,dt)

    # Store results
    x_history.append(x.flatten())
    x_hat_history.append(x_hat.flatten())
    error_history.append(error.flatten())
    u_history.append(u.flatten())
    r_history.append(r.flatten())  # Store the reference signal

# Convert lists to arrays for plotting
x_history = np.array(x_history)
x_hat_history = np.array(x_hat_history)
e_history = np.array(error_history)
u_history = np.array(u_history)
r_history = np.array(r_history)

# Plot the results
plt.figure()

# Plot true states and estimated states
# plt.subplot(4, 1, 1)
plt.plot(t, x_history[:, 0], label="Plant position")
plt.plot(t, x_hat_history[:, 0], '--', label="Observer position")
plt.legend()
plt.ylabel('angular displacement')
plt.xlabel('time')

plt.figure()
# plt.subplot(4, 1, 2)
plt.plot(t, x_history[:, 1], label="Plant velocity")
plt.plot(t, x_hat_history[:, 1], '--', label="Observer velocity")
plt.legend()
plt.ylabel('angular velocity')
plt.xlabel('time')

plt.figure()
# plt.subplot(4, 1, 3)
plt.plot(t, e_history[:,1], label="error")
plt.legend()
plt.ylabel('error')
plt.xlabel('time')

# Plot control input
plt.figure()
# plt.subplot(4, 1, 3)
plt.plot(t, u_history, label="Control Input u")
plt.legend()
plt.ylabel('Control Input')
plt.xlabel('time')

# Plot reference signal (sine wave)
plt.figure()
# plt.subplot(4, 1, 4)
plt.plot(t, r_history, label="Reference Input r (Sine Wave)")
plt.legend()
plt.ylabel('Reference Input')
plt.xlabel('Time (s)')


plt.show()
