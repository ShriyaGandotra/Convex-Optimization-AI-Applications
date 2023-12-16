import cvxpy as cp
import numpy as np
import matplotlib.pyplot as plt

def create_smart_charging_model(T, V, V_k, t_0, n, eta_bi, Delta_T, E_i_0, E_i_max, p_i_max, tau_ai, tau_di, n_s_i, n_p_i, eta_ci, p_k_max, max_import_limit):
    # Decision variables
    p_oi = cp.Variable((len(V), len(T)))
    E_i = cp.Variable(len(V))
    p_Vk = cp.Variable((len(V_k), len(T)))

    # Objective function
    objective = cp.Maximize(cp.sum([(t + t_0 + 1 - tau_ai[i]) * eta_bi[i] * p_oi[i, t] for t in tau for i in range(len(V))]))
 
    

    # Constraints
    constraints = []
    for i in range(len(V)):
        # Energy balance for each vehicle
        constraints += [(eta_bi[i] * Delta_T * cp.sum(p_oi[i, :]) + E_i[i] <= E_i_0[i])]

        # Charging power limits for each vehicle
        for t in tau:
            if tau_ai[i] <= t <= tau_di[i]:
                constraints += [0 <= p_oi[i, t], p_oi[i, t] <= p_i_max[i]]
                
            else:
                constraints += [p_oi[i, t] == 0]
   


    for k, phases in V_k.items():
        # Power flow on each phase k of the distribution network
        constraints += [p_Vk[k, :] == cp.sum([(n_s_i[i] * n_p_i[i]) / eta_ci[i] * p_oi[i, :] for i in range(len(V)) if k in phases])]

        # Maximum power flow limits for each phase k
        constraints += [p_Vk[k, :] <= p_k_max[k]]

    # Total import limit constraint
    total_import_limit = cp.sum(cp.sum(p_oi, axis=0))
    constraints += [total_import_limit <= max_import_limit]
    

    # Create the problem and solve
    problem = cp.Problem(objective, constraints)
    problem.solve(solver=cp.ECOS)

    # Return the results
    return problem, p_oi.value, E_i.value, p_Vk.value

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
tau = {i: np.arange(tau_ai[i], tau_di[i], 1) for i in V}
n_s_i = {i: 100 for i in V}    # Example series
n_p_i = {i:75 for i in V}
#n_p_i = {i: np.random.choice([20, 40, 60], p=[0.4, 0.4, 0.2]) for i in V}  # Random battery capacities
eta_ci = {i: 0.95 for i in V}  # Example charging efficiency
p_k_max = {0: 120, 1: 120, 2: 120}  # Example maximum power flow limits for each phase
max_import_limit = 120  # Maximum import limit for the charging station
smoothing_factor = 0.01
battery_capacity = 2.2  # Ah
c_rate = 1  # 1C
soc = 0.5  # 50% SoC
soc_threshold = 0.8  # 80% SoC
voltage_nominal = 4.2  # V
internal_resistance = 0.05  # Ohms

# Assume we start the CV phase at 80% SoC
soc_start_cv = 0.8

def calculate_charging_current(battery_capacity, c_rate, soc, soc_threshold, voltage_nominal, internal_resistance):
    """
    Calculates the charging current for a battery given its state of charge (SoC) and capacity.
    
    :param battery_capacity: Capacity of the battery in ampere-hours (Ah)
    :param c_rate: Charging rate as a multiple of the battery's capacity (1C, 0.5C, etc.)
    :param soc: Current state of charge of the battery as a fraction (0.0 to 1.0)
    :param soc_threshold: The SoC value at which the charger switches from CC to CV phase
    :param voltage_nominal: Nominal voltage of the battery during CV phase
    :param internal_resistance: Internal resistance of the battery in ohms
    :return: The charging current in amperes (A)
    """
    
    # During the constant current (CC) phase
    if soc < soc_threshold:
        return battery_capacity * c_rate
    
    # During the constant voltage (CV) phase
    else:
        # Calculate the maximum possible current without exceeding the nominal voltage
        voltage_drop = internal_resistance * (battery_capacity * c_rate)
        current_cv = (voltage_nominal - voltage_drop) / internal_resistance
        
        # The current will decrease as the battery charges, so we limit it to the calculated value
        return min(battery_capacity * c_rate, current_cv)

    
print(tau_ai)
print(tau_di)
print(eta_bi)

# Call the create_smart_charging_model function with the parameters
model, p_oi, E_i, p_Vk = create_smart_charging_model(T, V, V_k, t_0, n, eta_bi, Delta_T, E_i_0, E_i_max, p_i_max, tau_ai, tau_di, n_s_i, n_p_i, eta_ci, p_k_max, max_import_limit)

# Print or visualize the results as needed
# For example, print the values of p_oi, E_i, and p_Vk
print("Charging Power:")
print(p_oi)
print("\nState of Charge:")
print(E_i)
print("\nPower Flow:")
print(p_Vk)

# Simulating State of Charge (SoC)
E_i_simulated = np.array([np.linspace(0, 100, len(T)) for _ in V])

# Simulating Power Flow (p_Vk)
p_Vk_simulated = np.random.uniform(10, 30, (3, len(T)))  # 3 phases

# Simulating Charging Power (p_oi)
p_oi_simulated = np.random.uniform(0, 10, (len(V), len(T)))

# Plotting the simulations

# Plotting State of Charge vs Time
plt.figure(figsize=(12, 8))

plt.subplot(3, 1, 1)
for i in range(len(V)):
    plt.plot(T, E_i_simulated[i], label=f'Vehicle {i}')
plt.xlabel('Time')
plt.ylabel('State of Charge')
plt.title('State of Charge vs Time')
plt.legend()
plt.grid(True)

# Plotting Power Flow vs Time
plt.subplot(3, 1, 2)
for k in range(p_Vk_simulated.shape[0]):
    plt.plot(T, p_Vk_simulated[k], label=f'Phase {k}')
plt.xlabel('Time')
plt.ylabel('Power Flow')
plt.title('Power Flow vs Time')
plt.legend()
plt.grid(True)

# Plotting Charging Power vs Time
plt.subplot(3, 1, 3)
for i in range(len(V)):
    plt.plot(T, p_oi_simulated[i], label=f'Vehicle {i}')
plt.xlabel('Time')
plt.ylabel('Charging Power')
plt.title('Charging Power vs Time')
plt.legend()
plt.grid(True)


# Sum power across all vehicles for each time step
total_power = np.sum(p_oi, axis=0)

# Plot the total power vs Time
plt.figure(figsize=(12, 6))
plt.plot(T, total_power, label='Total Power', linewidth=2)
plt.xlabel('Time (h)')
plt.ylabel('Power (kW)')
plt.title('Total Power vs Time')
plt.legend()
plt.grid(True)
plt.show()


# Sigmoid function to simulate the SoC curve
def sigmoid(t, a=70, b=12, c=0.4, d=0):
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

plt.tight_layout()
plt.show()
