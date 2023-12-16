import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt
import random

def create_cvxpy_model(T, V, V_k, t_0, n, eta_bi, Delta_T, E_i_0, E_i_max, p_i_max, tau_ai, tau_di, n_s_i, n_p_i, eta_ci, p_k_max, v_oi, avi, bvi):
    # Decision variables
    p_bi = cp.Variable((len(V), len(T)))
    p_oi = cp.Variable((len(V), len(T)))
    E_i = cp.Variable(len(V))
    p_Vk = cp.Variable((len(V_k), len(T)))

    # Objective function
    objective = cp.Maximize(cp.sum([(t + t_0 + 1 - tau_ai[i]) * p_bi[i, t] for t in T for i in range(len(V))]))

    # Constraints
    constraints = []
    for i in range(len(V)):
        # Energy balance for each vehicle
       
        constraints += [(Delta_T * cp.sum(p_bi[i, :]) + E_i[i] <= E_i_0[i])]

        # Energy balance for each vehicle while charging
        constraints += [(eta_bi[i] * Delta_T * cp.sum(p_oi[i, :]) + E_i[i] <= E_i_0[i])]

        # Charging power limits for each vehicle
        for t in T:
            if tau_ai[i] <= t <= tau_di[i]:
                constraints += [p_oi[i, t] == 0]
            else:
                constraints += [0 <= p_oi[i, t], p_oi[i, t] <= p_i_max[i]]
        print (i)
    for k, phases in V_k.items():
        # Power flow on each phase k of the distribution network
        constraints += [p_Vk[k, :] == cp.sum([(n_s_i[i] * n_p_i[i]) / eta_ci[i] * p_oi[i, :] for i in range(len(V)) if k in phases])]

        # Maximum power flow limits for each phase k
        constraints += [p_Vk[k, :] <= p_k_max[k]]

    # Second-order cone constraints
    for i in range(len(V)):
        for t in T:
            print (voi)
            #constraints += [[cp.norm(cp.hstack([2 * (Ati[i, t] * p_bi[i, t] + v0i[i]), (((voi[i] * Ati[i, t]) + (Ri*et.T[:, t]))*p_bi[i,t]) + (voi[i] * v0i[i]) - 1]))] <= ((((voi[i]*Ati[i,t]) + (Ri*et.T[:, t]))* p_bi[i,t])+ (voi[i] * v0i[i]) + 1)]
            #constraints += [[cp.norm(cp.hstack([2 * (Ati[i, t] * p_bi[i, t] + v0i[i]), (((voi[i] * Ati[i, t]) + (Ri * et.T[:, t])) * p_bi[i, t]) + (voi[i] * v0i[i]) - 1]))] <= ((((voi[i] * Ati[i, t]) + (Ri * et.T[:, t])) * p_bi[i, t]) + (voi[i] * v0i[i]) + 1)]
            lhsa = cp.norm(cp.hstack([2 * (Ati[i, t] * p_bi[i, t] + v0i[i]), 
                                 (voi[i] * Ati[i, t] + Ri * et[:, t]) * p_bi[i, t] + voi[i] * v0i[i] - 1]))

            # Right-hand side of the second-order cone constraint
            rhsa = (voi[i] * Ati[i, t] + Ri * et[:, t]) * p_bi[i, t] + voi[i] * v0i[i] + 1

            # Adding the SOCP constraint to the list of constraints
            constraints.append(lhsa <= rhsa)
            LHS = cp.norm(cp.hstack([
            2 * cp.sqrt(Ri) * et[t] * p_bi[i, t],
            et[t] * (p_bi[i, t] - p_oi[i, t]) + voi[i] * (Ati[i, t] * p_bi[i, t] + v0i[i])
        ]))

        # Constructing the right-hand side (RHS) of the inequality
        RHS = et[t] * (p_oi[i, t] - p_bi[i, t]) + voi[i] * (Ati[i, t] * p_bi[i, t] + v0i[i])

        # Adding the SOCP constraint to the list of constraints
        constraints += [LHS <= RHS]
    # Create the problem and solve
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.ECOS)

    # Return the results
    return problem, p_bi.value, p_oi.value, E_i.value, p_Vk.value

  
# Example usage
T = range(0, 24)  # Example time range
V = range(0, 10)  # Example vehicle range (50 vehicles)
V_k = {0: [1, 2], 1: [2, 3, 4], 2: [4, 5]}  # Example mapping of vehicles to phases
t_0 = 0           # Example starting time
n = len(V)        # Number of vehicles
o = len(T)
eta_bi = {i: 0.91 for i in V}  # Example charging efficiency
Delta_T = 5        # Example time step
E_i_0 = {i: np.random.uniform(0.1, 0.4) * 60 for i in V}  # Random initial energy levels between 10% and 40%
E_i_max = {20: 20, 40: 40, 60: 60}  # Example maximum energy levels for different battery capacities
p_i_max = {i: 50 for i in V}   # Example maximum charging power for all vehicles
tau_ai = {i: np.random.uniform(6 * 60, 18 * 60) for i in V}  # Random arrival times between 6 A.M. and 6 P.M.
tau_di = {i: tau_ai[i] + np.random.uniform(15, 4 * 60) for i in V}  # Random durations between 15 min and 4 h
n_s_i = {i: 100 for i in V}    # Example series
n_p_i = {i: np.random.choice([20, 40, 60], p=[0.4, 0.4, 0.2]) for i in V}  # Random battery capacities
eta_ci = {i: 0.95 for i in V}  # Example charging efficiency
p_k_max = {0: 120, 1: 120, 2: 120}  # Example maximum power flow limits for each phase
max_import_limit = 120  # Maximum import limit for the charging station
avi = [67.92 for i in V]  
bvi = [3.592 for i in V]  # Coefficient for the linear relationship
Ei_t0 = 8.14 # Stored energy at t0 for each vehicle
B=24
v_oi = 4.15


# Variables initialization
v0i = np.array([avi[i] * Ei_t0 + bvi[i] for i in V])  # Use range(Nv) for iteration
voi = np.random.uniform(300, 400, len(V))  # Nv is the size of the array you want to generate
#Ri = np.array([148 for i in V])  # Make Ri an array if it should be different for each vehicle
Ri = 148
#Ati = np.array([[avi[i] * B * (t - t_0)] for t in T for i in V]) # Ati calculation
Ati = np.array([[avi[i] * B * (t - t_0) for t in T] for i in V])  # Assuming avi is a constant
et = np.eye(B) # Binary matrix for selecting time steps
# Call the create_cvxpy_model function with the parameters
model, p_bi, p_oi, E_i, p_Vk = create_cvxpy_model(T, V, V_k, t_0, n, eta_bi, Delta_T, E_i_0, E_i_max, p_i_max, tau_ai, tau_di, n_s_i, n_p_i, eta_ci, p_k_max, v_oi, avi, bvi)

# Print or visualize the results as needed
print("Charging Power:")
print(p_bi)
print("\nState of Charge:")
print(E_i)
print("\nPower Flow:")
print(p_Vk)


# Simulating State of Charge (SoC)
#E_i_simulated = np.array([np.linspace(E_i_0[i], E_i_max[i], len(T)) for i in V])
E_i_simulated = np.array([np.linspace(0, 100, len(T)) for _ in V])

# Simulating Power Flow (p_Vk)
p_Vk_simulated = np.random.uniform(10, 30, (len(V_k), len(T)))  # For each phase

# Simulating Charging Power (p_oi)
p_bi_simulated = np.random.uniform(0, 50, (len(V), len(T)))  # For each vehicle

# Plotting the simulations

# Plotting State of Charge vs Time
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
for i in V:
    plt.plot(T, E_i_simulated[i], label=f'Vehicle {i}')
plt.xlabel('Time (hours)')
plt.ylabel('State of Charge (kWh)')
plt.title('State of Charge vs Time')
plt.legend()
plt.grid(True)

# Plotting Power Flow vs Time
plt.subplot(3, 1, 2)
for k in V_k:
    plt.plot(T, p_Vk_simulated[k], label=f'Phase {k}')
plt.xlabel('Time (hours)')
plt.ylabel('Power Flow (kW)')
plt.title('Power Flow vs Time')
plt.legend()
plt.grid(True)

# Plotting Charging Power vs Time
plt.subplot(3, 1, 3)
for i in V:
    plt.plot(E_i_simulated[i], p_bi_simulated[i], label=f'Vehicle {i}')
plt.xlabel('Time (hours)')
plt.ylabel('Charging Power (kW)')
plt.title('Charging Power vs Time')
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

# Sigmoid function to simulate the SoC curve
def sigmoid(t, a=85, b=12, c=0.4, d=0):
    return a / (1 + np.exp(-c * (t - b))) + d

# Create an array of time points over which to evaluate the SoC
time_points = np.linspace(0, 24, 200)  # Cover a full day

# Calculate the SoC using the sigmoid function with initial parameter guesses
SoC_values = sigmoid(time_points)

# Find the time point where SoC first reaches 70% to adjust 'c' accordingly
# This is an iterative process - you may need to run this section multiple times
for t, soc in zip(time_points, SoC_values):
    if soc >= 70:
        print(f"SoC reaches 70% at t={t}")
        break

# Plot the SoC curve with the initial parameter guesses
plt.figure(figsize=(10, 5))
plt.plot(time_points, SoC_values, label='SoC Curve')
#plt.axvline(x=6, color='grey', linestyle='--', label='Start of Charging (t=6)')
#plt.axhline(y=70, color='red', linestyle='--', label='Target SoC 70%')
#plt.axvline(x=18, color='green', linestyle='--', label='Target Time (t=18)')
plt.xlabel('Time')
plt.ylabel('State of Charge (%)')
plt.title('Simulated State of Charge Curve')
plt.legend()
plt.grid(True)
plt.show()
