import numpy as np
from scipy.optimize import fsolve
import matplotlib.pyplot as plt
import pandas as pd

# Constants
psi_to_pa = 6894.76
Cd_needle = 0.7
Cd_ball = 1
Cd_manifold = 0.85
Cd_injector = 0.75
n_ball = 3
Cd_effective = Cd_needle * (Cd_ball ** n_ball) * Cd_manifold * Cd_injector

# Fluid properties
density_nitrous = 720  # kg/m^3
viscosity_nitrous = 1.8e-3  # Pa.s
density_ethanol = 789  # kg/m^3
viscosity_ethanol = 1.2e-3  # Pa.s
roughness = 5e-6  # m

# Darcy friction factor
def darcy_friction_factor(Re, roughness, D):
    if Re < 2000:
        return 64 / Re
    elif Re < 4000:
        f_lam = 64 / Re
        f_turb = 0.25 / (np.log10(roughness / (3.7 * D) + 5.74 / Re**0.9))**2
        return (f_lam + f_turb) / 2
    else:
        return 0.25 / (np.log10(roughness / (3.7 * D) + 5.74 / Re**0.9))**2

# Pressure drop function
def pressure_drop_hose(L, D, density, viscosity, Q, n_parallel=1):
    A = np.pi * (D / 2)**2
    velocity = (Q / n_parallel) / (density * A)
    Re = density * velocity * D / viscosity
    f = darcy_friction_factor(Re, roughness, D)
    dP = f * (L / D) * 0.5 * density * velocity**2
    return dP, velocity, Re

# Mass flow solver
def calculate_mass_flow(d_valve, P_inlet_psi, P_chamber_psi, density, L, D, viscosity, branches):
    P_inlet, P_chamber = P_inlet_psi * psi_to_pa, P_chamber_psi * psi_to_pa
    A_valve, Q = np.pi * (d_valve / 2)**2, 0.1
    velocities, Reynolds = [], []

    for iteration in range(50):
        dP_eff = P_inlet - P_chamber
        velocities.clear()
        Reynolds.clear()
        
        for L_seg, D_seg, n in zip(L, D, branches):
            dP, v, Re = pressure_drop_hose(L_seg, D_seg, density, viscosity, Q, n)
            dP_eff -= dP
            velocities.append(v)
            Reynolds.append(Re)
        
        if dP_eff <= 0:
            print("Warning: Negative effective pressure drop encountered.")
            return 0, velocities, Reynolds

        v_valve = Cd_effective * np.sqrt(2 * dP_eff / density)
        Q_new = density * A_valve * v_valve
        velocities.append(v_valve)

        if abs(Q - Q_new) < 1e-5:
            return Q_new, velocities, Reynolds
        Q = Q_new

    print("Warning: Solver did not converge within 50 iterations.")
    return Q, velocities, Reynolds

# Valve diameter solver
def calculate_valve_diameter(target_Q, P_inlet_psi, P_chamber_psi, density, L, D, viscosity, branches):
    initial_guess = 0.005
    solution, = fsolve(lambda d: calculate_mass_flow(d, P_inlet_psi, P_chamber_psi, density, L, D, viscosity, branches)[0] - target_Q,
                       initial_guess)
    return solution

# Hose configuration
hose_lengths_nitrous = [0.5, 0.1524]
hose_diameters_nitrous = [0.00635, 0.00635]
parallel_branches_nitrous = [1, 3]

hose_lengths_ethanol = [.5]
hose_diameters_ethanol = [0.00635]
parallel_branches_ethanol = [1]

P_valve_inlet_psi = 825


# Target mass flows
target_mass_flow_nitrous = 0.467
target_mass_flow_ethanol = 0.233

# Chamber pressures
chamber_pressures = np.linspace(14.5, 600, 50)
results = []

for P_chamber_psi in chamber_pressures:
    d_valve_nitrous = calculate_valve_diameter(target_mass_flow_nitrous, P_valve_inlet_psi, P_chamber_psi,
                                               density_nitrous, hose_lengths_nitrous, hose_diameters_nitrous,
                                               viscosity_nitrous, parallel_branches_nitrous)
    Q_nitrous, velocities_nitrous, _ = calculate_mass_flow(d_valve_nitrous, P_valve_inlet_psi, P_chamber_psi,
                                                           density_nitrous, hose_lengths_nitrous,
                                                           hose_diameters_nitrous, viscosity_nitrous,
                                                           parallel_branches_nitrous)

    d_valve_ethanol = calculate_valve_diameter(target_mass_flow_ethanol, P_valve_inlet_psi, P_chamber_psi,
                                               density_ethanol, hose_lengths_ethanol, hose_diameters_ethanol,
                                               viscosity_ethanol, parallel_branches_ethanol)
    Q_ethanol, velocities_ethanol, _ = calculate_mass_flow(d_valve_ethanol, P_valve_inlet_psi, P_chamber_psi,
                                                           density_ethanol, hose_lengths_ethanol,
                                                           hose_diameters_ethanol, viscosity_ethanol,
                                                           parallel_branches_ethanol)

    results.append([P_chamber_psi, d_valve_nitrous*1000, Q_nitrous, velocities_nitrous[-1],
                    d_valve_ethanol*1000, Q_ethanol, velocities_ethanol[-1]])

# DataFrame for results
df_results = pd.DataFrame(results, columns=[
    'Chamber Pressure (psi)', 'Nitrous Valve Dia (mm)', 'Nitrous Mass Flow (kg/s)', 'Nitrous Exit Vel (m/s)',
    'Ethanol Valve Dia (mm)', 'Ethanol Mass Flow (kg/s)', 'Ethanol Exit Vel (m/s)'
])

# Set pandas display options (Insert these lines here)
pd.set_option('display.max_columns', None)  # Display all columns
pd.set_option('display.expand_frame_repr', False)  # Prevent line breaks in output

print(df_results)

# Quadratic fit and plot
coeffs_n2o = np.polyfit(chamber_pressures, df_results['Nitrous Valve Dia (mm)'], 2)
coeffs_ethanol = np.polyfit(chamber_pressures, df_results['Ethanol Valve Dia (mm)'], 2)

plt.figure(figsize=(12,7))
plt.plot(chamber_pressures, df_results['Nitrous Valve Dia (mm)'], label="Nitrous Oxide")
plt.plot(chamber_pressures, np.polyval(coeffs_n2o, chamber_pressures), '--', label=f"N2O Fit: {coeffs_n2o[0]:.2e}x²+{coeffs_n2o[1]:.2e}x+{coeffs_n2o[2]:.2e}")
plt.plot(chamber_pressures, df_results['Ethanol Valve Dia (mm)'], label="Ethanol")
plt.plot(chamber_pressures, np.polyval(coeffs_ethanol, chamber_pressures), '--', label=f"Ethanol Fit: {coeffs_ethanol[0]:.2e}x²+{coeffs_ethanol[1]:.2e}x+{coeffs_ethanol[2]:.2e}")

plt.xlabel("Chamber Pressure (psi)")
plt.ylabel("Valve Diameter (mm)")
plt.title("Valve Diameter vs Chamber Pressure")
plt.legend()
plt.grid(True)
plt.tight_layout()
plt.show()

