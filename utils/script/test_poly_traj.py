import numpy as np
import matplotlib.pyplot as plt

def getSegCoeff(T0, T1, b):
    coeff = np.zeros(6)
    A = np.zeros((6, 6))

    A[0, :] = [T0**5, T0**4, T0**3, T0**2, T0, 1]
    A[1, :] = [T1**5, T1**4, T1**3, T1**2, T1, 1]
    A[2, :] = [5*T0**4, 4*T0**3, 3*T0**2, 2*T0, 1, 0]
    A[3, :] = [5*T1**4, 4*T1**3, 3*T1**2, 2*T1, 1, 0]
    A[4, :] = [20*T0**3, 12*T0**2, 6*T0, 2, 0, 0]
    A[5, :] = [20*T1**3, 12*T1**2, 6*T1, 2, 0, 0]

    coeff = np.linalg.solve(A, b)
    return coeff


t_vals = []
x_vals, vx_vals, ax_vals = [], [], []
y_vals, vy_vals, ay_vals = [], [], []

dt = 0.02
T = np.array([2.0, 1.0, 1.0, 2.0])
        
y_all = np.array([0.0, 5.0, 10.0, 15.0, 20.0])
vy_all = np.array([0.0, 5.0, 5.0, 5.0, 0.0])
ay_all = np.array([0.0, 0.0, 0, 0.0, 0.0])

x_all = np.array([0.0, 0.0, 0.0])
vx_all = np.array([0.0, 0.0, 0.0])
ax_all = np.array([0.0, 0.0, 0.0])

# x_all = np.array([0.0, -2.0, 0.0])
# vx_all = np.array([0.0, 0.0, 0.0])
# ax_all = np.array([0.0, 1.5, 0.0])


for t in np.arange(0.0, 6.0, dt):
    if (t < T[0]):
        by = np.array([y_all[0], y_all[1], vy_all[0], vy_all[1], ay_all[0], ay_all[1]])
        y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(0, T[0], by))
        vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(0, T[0], by))
        ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(0, T[0], by))
    elif (t < T[0] + T[1]):
        by = np.array([y_all[1], y_all[2], vy_all[1], vy_all[2], ay_all[1], ay_all[2]])
        y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0], T[0] + T[1], by))
        vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0], T[0] + T[1], by))
        ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0], T[0] + T[1], by))
    elif (t < T[0] + T[1] + T[2]):
        by = np.array([y_all[2], y_all[3], vy_all[2], vy_all[3], ay_all[2], ay_all[3]])
        y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
        vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
        ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2], by))
    elif (t < T[0] + T[1] + T[2] + T[3]):
        by = np.array([y_all[3], y_all[4], vy_all[3], vy_all[4], ay_all[3], ay_all[4]])
        y = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
        vy = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
        ay = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1] + T[2], T[0] + T[1] + T[2] + T[3], by))
    
    if (t < T[0] + T[1]):
        bx = np.array([x_all[0], x_all[1], vx_all[0], vx_all[1], ax_all[0], ax_all[1]])
        x = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(0, T[0] + T[1], bx))
        vx = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(0, T[0] + T[1], bx))
        ax = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(0, T[0] + T[1], bx))
    elif (t < T[0] + T[1] + T[2] + T[3]):
        bx = np.array([x_all[1], x_all[2], vx_all[1], vx_all[2], ax_all[1], ax_all[2]])
        x = np.dot([t**5, t**4, t**3, t**2, t, 1], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))
        vx = np.dot([5*t**4, 4*t**3, 3*t**2, 2*t, 1, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))
        ax = np.dot([20*t**3, 12*t**2, 6*t, 2, 0, 0], getSegCoeff(T[0] + T[1], T[0] + T[1] + T[2] + T[3], bx))

    # Append values to the arrays
    x_vals.append(x)
    y_vals.append(y)
    vx_vals.append(vx)
    vy_vals.append(vy)
    ax_vals.append(ax)
    ay_vals.append(ay)
    t_vals.append(t)

# # Create the plot
fig, axs = plt.subplots(6, 1, sharex=True, figsize=(8,8))

axs[0].plot(t_vals, x_vals)
axs[0].set_ylabel("Position (m)")

axs[1].plot(t_vals, vx_vals)
axs[1].set_ylabel("Velocity (m/s)")

axs[2].plot(t_vals, ax_vals)
axs[2].set_ylabel("Acceleration (m/s^2)")
axs[2].set_xlabel("Time (s)")

axs[3].plot(t_vals, y_vals)
axs[3].set_ylabel("Position (m)")

axs[4].plot(t_vals, vy_vals)
axs[4].set_ylabel("Velocity (m/s)")

axs[5].plot(t_vals, ay_vals)
axs[5].set_ylabel("Acceleration (m/s^2)")

# plt.show()

plt.figure()
plt.plot(x_vals,y_vals)
plt.show()