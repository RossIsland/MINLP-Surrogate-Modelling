from amplpy import AMPL
ampl = AMPL()

ampl.reset()

ampl.eval(
    """
#VARIABLES
#-----------
param alpha_e := 0.1229;  # in kgce/kwh
param alpha_g := 1.3300;  # in kgce/m^3
var W_comp {1..5}  >= 0.0;  # Daily electric energy tracking component
var fg_comp6       >= 0.0;  # Daily fuel gas consumption volume component

# State Bound Variables (Relaxed bounds to give new multi-variable planes math space)
var P_node {1..18} >= 1.0, <= 12.0;     # Pressure variables (MPa)
var T_node {1..18} >= 200.0, <= 380.0; # Temperature variables (K)

# Compressor Engineering Control Variables
var s_comp {1..6}  >= 2880.0, <= 6405.0; # Rotational Speed (r/min)
var n_comp {1..6}  >= 0, <= 4, integer;   # Active running units

# Performance Bounds Variables
var Q_min {1..6}   >= 0.0;                # Surge limit flow targets
var Q_max {1..6}   >= 0.0;                # Choke limit flow targets

param d       := 1.0;     # Internal pipeline diameter (m)
param L       := 50000.0; # Segment length (m)
param delta_h := 15.0;     # Node elevation profile difference (m)

var NQ_in1;

# Volumetric flowrate declaration
var Q_pipe {1..11} >= 0.0, <= 8000.0;   # Gas flow bounds expanded for table units
var Q_comp {1..6}  >= 0.0, <= 8000.0;   # Gas flow bounds expanded for table units

# FIXED: Removed the /86400 scaling inflation to match your unit-corrected ANN weights
param demand_n4  := 10.8941;  
param demand_n6  := 16.5385;  
param demand_n9  := 213.0248; 
param demand_n11 := 3.8957;  
param demand_n12 := 7.4863;  
param demand_n14 := 94.9072; 
param demand_n17 := 259.3763; 
param demand_n18 := 6193.7378; # Raw baseline matching Table 4

#OBJECTIVE FUNCTION
#-------------------
minimize total_daily_energy_consumption:
    ((24.0 * alpha_e * sum {m in 1..5} W_comp[m]) + (86400 * alpha_g * fg_comp6)) / 1000.0;   # Native output in tce/day

    
#CONSTRAINTS
#------------
# Constraint category 1
subject to source_node: -Q_pipe[1] = NQ_in1;
subject to supply_capacity_bound: 0 <= -NQ_in1 <= 8000.0;

# Entry and Exit Anchors to bind system network pressure bounds profiles
subject to anchor_P1:  P_node[1] == 6.0;      # Base source pipeline pressure (MPa)
subject to anchor_T1:  T_node[1] == 293.15;   # Base gas entry temperature (Kelvin)
subject to anchor_P18: P_node[18] >= 1.8;     # Terminal target delivery limit

subject to balance_node_2:  Q_pipe[1] - Q_comp[1] = 0;
subject to balance_node_3:  Q_comp[1] - Q_pipe[2] = 0;
subject to balance_node_4:  Q_pipe[2] - Q_comp[2] = demand_n4;
subject to balance_node_5:  Q_comp[2] - Q_pipe[3] = 0;
subject to balance_node_6:  Q_pipe[3] - Q_pipe[4] = demand_n6;
subject to balance_node_7:  Q_pipe[4] - Q_comp[3] = 0;
subject to balance_node_8:  Q_comp[3] - Q_pipe[5] = 0;
subject to balance_node_9:  Q_pipe[5] - Q_comp[4] = demand_n9;
subject to balance_node_10: Q_comp[4] - Q_pipe[6] = 0;
subject to balance_node_11: Q_pipe[6] - Q_pipe[7] = demand_n11;
subject to balance_node_12: Q_pipe[7] - Q_comp[5] = demand_n12;
subject to balance_node_13: Q_comp[5] - Q_pipe[8] = 0;
subject to balance_node_14: Q_pipe[8] - Q_pipe[9] = demand_n14;
subject to balance_node_15: Q_pipe[9] - Q_comp[6] = 0;
subject to balance_node_16: Q_comp[6] - Q_pipe[10] = 0;
subject to balance_node_17: Q_pipe[10] - Q_pipe[11] = demand_n17;
subject to balance_node_18: Q_pipe[11] = demand_n18; 

#category 2 & 3 constraints
#between node 2 and node 3 compressor section
subject to comp_1_pressure_ratio:
    P_node[3] = (1.134228e+00 * P_node[2]) + (8.691296e-03 * T_node[2]) + (-5.074929e-05 * Q_comp[1]) + (8.641332e-04 * s_comp[1]) + (1.620426e-01 * n_comp[1]) - 5.448995;
subject to comp_1_temperature_ratio:
    T_node[3] = (8.102546e-01 * P_node[2]) + (9.516915e-01 * T_node[2]) + (-4.934076e-04 * Q_comp[1]) + (6.172136e-03 * s_comp[1]) + (3.088561e-01 * n_comp[1]) - 0.266773;
subject to comp_1_volumetric_outflow:
    Q_pipe[2] = (-8.412206e+01 * P_node[2]) + (-1.644402e+01 * T_node[2]) + (7.843703e-01 * Q_comp[1]) + (-7.549419e-01 * s_comp[1]) + (-2.824816e+02 * n_comp[1]) + 11181.502490;
subject to comp_1_shaft_work:
    W_comp[1] = (-6.357542e+02 * P_node[2]) + (9.603108e+01 * T_node[2]) + (3.087110e+00 * Q_comp[1]) + (2.040797e+01 * s_comp[1]) + (2.241769e+03 * n_comp[1]) -114000.000000;
subject to comp_1_surge_boundary:
    Q_min[1]  = (4.801316e+00 * P_node[2]) + (-5.855858e-02 * T_node[2]) + (-2.804919e-04 * Q_comp[1]) + (8.005890e-02 * s_comp[1]) + (6.921217e-01 * n_comp[1]) + 35.593123;
subject to comp_1_choke_boundary:
    Q_max[1]  = (1.276972e+02 * P_node[2]) + (1.967430e+01 * T_node[2]) + (-4.882471e-02 * Q_comp[1]) + (3.450256e+00 * s_comp[1]) + (1.396603e+02 * n_comp[1]) - 5371.834328;

#between node 3 and node 4
subject to pipe_2_pressure:
    P_node[4] = (1.187822e+00 * P_node[3]) + (2.825055e-02 * T_node[3]) + (-1.647939e-05 * Q_pipe[2]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_2_temperature:
    T_node[4] = (2.385662e+00 * P_node[3]) + (9.103613e-01 * T_node[3]) + (3.415186e-05 * Q_pipe[2]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#between node 4 and node 5
subject to comp_2_pressure_ratio:
    P_node[5] = (1.134228e+00 * P_node[4]) + (8.691296e-03 * T_node[4]) + (-5.074929e-05 * Q_comp[2]) + (8.641332e-04 * s_comp[2]) + (1.620426e-01 * n_comp[2]) - 5.448995;
subject to comp_2_temperature_ratio:
    T_node[5] = (8.102546e-01 * P_node[4]) + (9.516915e-01 * T_node[4]) + (-4.934076e-04 * Q_comp[2]) + (6.172136e-03 * s_comp[2]) + (3.088561e-01 * n_comp[2]) - 0.266773;
subject to comp_2_volumetric_outflow:
    Q_pipe[3] = (-8.412206e+01 * P_node[4]) + (-1.644402e+01 * T_node[4]) + (7.843703e-01 * Q_comp[2]) + (-7.549419e-01 * s_comp[2]) + (-2.824816e+02 * n_comp[2]) + 11181.502490;
subject to comp_2_shaft_work:
    W_comp[2] = (-6.357542e+02 * P_node[4]) + (9.603108e+01 * T_node[4]) + (3.087110e+00 * Q_comp[2]) + (2.040797e+01 * s_comp[2]) + (2.241769e+03 * n_comp[2]) -121000.000000;
subject to comp_2_surge_boundary:
    Q_min[2]  = (4.801316e+00 * P_node[4]) + (-5.855858e-02 * T_node[4]) + (-2.804919e-04 * Q_comp[2]) + (8.005890e-02 * s_comp[2]) + (6.921217e-01 * n_comp[2]) + 35.593123;
subject to comp_2_choke_boundary:
    Q_max[2]  = (1.276972e+02 * P_node[4]) + (1.967430e+01 * T_node[4]) + (-4.882471e-02 * Q_comp[2]) + (3.450256e+00 * s_comp[2]) + (1.396603e+02 * n_comp[2]) - 5371.834328;

#between node 5 and node 6
subject to pipe_3_pressure:
    P_node[6] = (1.187822e+00 * P_node[5]) + (2.825055e-02 * T_node[5]) + (-1.647939e-05 * Q_pipe[3]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_3_temperature:
    T_node[6] = (2.385662e+00 * P_node[5]) + (9.103613e-01 * T_node[5]) + (3.415186e-05 * Q_pipe[3]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#between node 6 and node 7
subject to pipe_4_pressure:
    P_node[7] = (1.187822e+00 * P_node[6]) + (2.825055e-02 * T_node[6]) + (-1.647939e-05 * Q_pipe[4]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_4_temperature:
    T_node[7] = (2.385662e+00 * P_node[6]) + (9.103613e-01 * T_node[6]) + (3.415186e-05 * Q_pipe[4]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#between node 7 and node 8
subject to comp_3_pressure_ratio:
    P_node[8] = (1.134228e+00 * P_node[7]) + (8.691296e-03 * T_node[7]) + (-5.074929e-05 * Q_comp[3]) + (8.641332e-04 * s_comp[3]) + (1.620426e-01 * n_comp[3]) - 5.448995;
subject to comp_3_temperature_ratio:
    T_node[8] = (8.102546e-01 * P_node[7]) + (9.516915e-01 * T_node[7]) + (-4.934076e-04 * Q_comp[3]) + (6.172136e-03 * s_comp[3]) + (3.088561e-01 * n_comp[3]) - 0.266773;
subject to comp_3_volumetric_outflow:
    Q_pipe[5] = (-8.412206e+01 * P_node[7]) + (-1.644402e+01 * T_node[7]) + (7.843703e-01 * Q_comp[3]) + (-7.549419e-01 * s_comp[3]) + (-2.824816e+02 * n_comp[3]) + 11181.502490;
subject to comp_3_shaft_work:
    W_comp[3] = (-6.357542e+02 * P_node[7]) + (9.603108e+01 * T_node[7]) + (3.087110e+00 * Q_comp[3]) + (2.040797e+01 * s_comp[3]) + (2.241769e+03 * n_comp[3]) -121000.000000;
subject to comp_3_surge_boundary:
    Q_min[3]  = (4.801316e+00 * P_node[7]) + (-5.855858e-02 * T_node[7]) + (-2.082941e-04 * Q_comp[3]) + (8.005890e-02 * s_comp[3]) + (6.921217e-01 * n_comp[3]) + 35.593123;
subject to comp_3_choke_boundary:
    Q_max[3]  = (1.276972e+02 * P_node[7]) + (1.967430e+01 * T_node[7]) + (-4.882471e-02 * Q_comp[3]) + (3.450256e+00 * s_comp[3]) + (1.396603e+02 * n_comp[3]) - 5371.834328;

#between node 8 and 9
subject to pipe_5_pressure:
    P_node[9] = (1.187822e+00 * P_node[8]) + (2.825055e-02 * T_node[8]) + (-1.647939e-05 * Q_pipe[5]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_5_temperature:
    T_node[9] = (2.385662e+00 * P_node[8]) + (9.103613e-01 * T_node[8]) + (3.415186e-05 * Q_pipe[5]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#between node 9 and 10
subject to comp_4_pressure_ratio:
    P_node[10] = (1.134228e+00 * P_node[9]) + (8.691296e-03 * T_node[9]) + (-5.074929e-05 * Q_comp[4]) + (8.641332e-04 * s_comp[4]) + (1.620426e-01 * n_comp[4]) - 5.448995;
subject to comp_4_temperature_ratio:
    T_node[10] = (8.102546e-01 * P_node[9]) + (9.516915e-01 * T_node[9]) + (-4.934076e-04 * Q_comp[4]) + (6.172136e-03 * s_comp[4]) + (3.088561e-01 * n_comp[4]) - 0.266773;
subject to comp_4_volumetric_outflow:
    Q_pipe[6]  = (-8.412206e+01 * P_node[9]) + (-1.644402e+01 * T_node[9]) + (7.843703e-01 * Q_comp[4]) + (-7.549419e-01 * s_comp[4]) + (-2.824816e+02 * n_comp[4]) + 11181.502490;
subject to comp_4_shaft_work:
    W_comp[4]  = (-6.357542e+02 * P_node[9]) + (9.603108e+01 * T_node[9]) + (3.087110e+00 * Q_comp[4]) + (2.040797e+01 * s_comp[4]) + (2.241769e+03 * n_comp[4]) -121000.000000;
subject to comp_4_surge_boundary:
    Q_min[4]   = (4.801316e+00 * P_node[9]) + (-5.855858e-02 * T_node[9]) + (-2.804919e-04 * Q_comp[4]) + (8.005890e-02 * s_comp[4]) + (6.921217e-01 * n_comp[4]) + 35.593123;
subject to comp_4_choke_boundary:
    Q_max[4]   = (1.276972e+02 * P_node[9]) + (1.967430e+01 * T_node[9]) + (-4.882471e-02 * Q_comp[4]) + (3.450256e+00 * s_comp[4]) + (1.396603e+02 * n_comp[4]) - 5371.834328;

#between node 10 and 11
subject to pipe_6_pressure:
    P_node[11] = (1.187822e+00 * P_node[10]) + (2.825055e-02 * T_node[10]) + (-1.647939e-05 * Q_pipe[6]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_6_temperature:
    T_node[11] = (2.385662e+00 * P_node[10]) + (9.103613e-01 * T_node[10]) + (3.415186e-05 * Q_pipe[6]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

# Pipeline Segment 7 (Node 11 -> Node 12)
subject to pipe_7_pressure:
    P_node[12] = (1.187822e+00 * P_node[11]) + (2.825055e-02 * T_node[11]) + (-1.647939e-05 * Q_pipe[7]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_7_temperature:
    T_node[12] = (2.385662e+00 * P_node[11]) + (9.103613e-01 * T_node[11]) + (3.415186e-05 * Q_pipe[7]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#node 12 & 13
subject to comp_5_pressure_ratio:
    P_node[13] = (1.134228e+00 * P_node[12]) + (8.691296e-03 * T_node[12]) + (-5.074929e-05 * Q_comp[5]) + (8.641332e-04 * s_comp[5]) + (1.620426e-01 * n_comp[5]) - 5.448995;
subject to comp_5_temperature_ratio:
    T_node[13] = (8.102546e-01 * P_node[12]) + (9.516915e-01 * T_node[12]) + (-4.934076e-04 * Q_comp[5]) + (6.172136e-03 * s_comp[5]) + (3.088561e-01 * n_comp[5]) - 0.266773;
subject to comp_5_volumetric_outflow:
    Q_pipe[8]  = (-8.412206e+01 * P_node[12]) + (-1.644402e+01 * T_node[12]) + (7.843703e-01 * Q_comp[5]) + (-7.549419e-01 * s_comp[5]) + (-2.824816e+02 * n_comp[5]) + 11181.502490;
subject to comp_5_shaft_work:
    W_comp[5]  = (-6.357542e+02 * P_node[12]) + (9.603108e+01 * T_node[12]) + (3.087110e+00 * Q_comp[5]) + (2.040797e+01 * s_comp[5]) + (2.241769e+03 * n_comp[5]) -121000.000000;
subject to comp_5_surge_boundary:
    Q_min[5]   = (4.801316e+00 * P_node[12]) + (-5.855858e-02 * T_node[12]) + (-2.804919e-04 * Q_comp[5]) + (8.005890e-02 * s_comp[5]) + (6.921217e-01 * n_comp[5]) + 35.593123;
subject to comp_5_choke_boundary:
    Q_max[5]   = (1.276972e+02 * P_node[12]) + (1.967430e+01 * T_node[12]) + (-4.882471e-02 * Q_comp[5]) + (3.450256e+00 * s_comp[5]) + (1.396603e+02 * n_comp[5]) - 5371.834328;

#node 13 & 14
subject to pipe_8_pressure:
    P_node[14] = (1.187822e+00 * P_node[13]) + (2.825055e-02 * T_node[13]) + (-1.647939e-05 * Q_pipe[8]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_8_temperature:
    T_node[14] = (2.385662e+00 * P_node[13]) + (9.103613e-01 * T_node[13]) + (3.415186e-05 * Q_pipe[8]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

# Pipeline Segment 9 (Node 14 -> Node 15)
subject to pipe_9_pressure:
    P_node[15] = (1.187822e+00 * P_node[14]) + (2.825055e-02 * T_node[14]) + (-1.647939e-05 * Q_pipe[9]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_9_temperature:
    T_node[15] = (2.385662e+00 * P_node[14]) + (9.103613e-01 * T_node[14]) + (3.415186e-05 * Q_pipe[9]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#between node 15 and 16
subject to comp_6_pressure_ratio:
    P_node[16] = (1.702710e+00 * P_node[15]) + (-7.819046e-03 * T_node[15]) + (-8.932528e-04 * Q_comp[6]) + (1.216046e-03 * s_comp[6]) + (4.405679e-01 * n_comp[6]) - 4.646351;
subject to comp_6_temperature_ratio:
    T_node[16] = (1.164226e+00 * P_node[15]) + (1.225983e+00 * T_node[15]) + (-1.409495e-02 * Q_comp[6]) + (1.752047e-02 * s_comp[6]) + (7.171396e+00 * n_comp[6]) - 119.330046;
subject to comp_6_volumetric_outflow:
    Q_pipe[10] <= (-4.305059e+01 * P_node[15]) + (-1.274703e+00 * T_node[15]) + (1.233040e+00 * Q_comp[6]) + (-2.939647e-02 * s_comp[6]) + (4.027076e+01 * n_comp[6]) + 187.480230;
subject to comp_6_fuel_consumption:
    fg_comp6 = (-9.592126e-03 * P_node[15]) + (-2.268387e-03 * T_node[15]) + (6.537890e-05 * Q_comp[6]) + (3.071554e-04 * s_comp[6]) + (2.241358e-01 * n_comp[6]) - 0.777623;
subject to comp_6_surge_boundary:
    Q_min[6] = (3.591954e+00 * P_node[15]) + (-7.164792e-01 * T_node[15]) + (1.502999e-02 * Q_comp[6]) + (4.405983e-02 * s_comp[6]) + (1.115068e+02 * n_comp[6]) - 56.354660;
subject to comp_6_choke_boundary:
    Q_max[6] = (2.094998e+01 * P_node[15]) + (-1.501785e+01 * T_node[15]) + (1.932712e-01 * Q_comp[6]) + (8.720240e-01 * s_comp[6]) + (1.743204e+03 * n_comp[6]) - 569.041217;

#node 16 & 17
subject to pipe_10_pressure:
    P_node[17] = (1.187822e+00 * P_node[16]) + (2.825055e-02 * T_node[16]) + (-1.423819e-04 * Q_pipe[10]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_10_temperature:
    T_node[17] = (2.385662e+00 * P_node[16]) + (9.103613e-01 * T_node[16]) + (3.415186e-05 * Q_pipe[10]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

# Pipeline Segment 11 (Node 17 -> Node 18)
subject to pipe_11_pressure:
    P_node[18] = (1.187822e+00 * P_node[17]) + (2.825055e-02 * T_node[17]) + (-1.423819e-04 * Q_pipe[11]) + (1.859381e+00 * d) + (-1.113134e-06 * L) + (-2.395774e-04 * delta_h) + (-11.702359);
subject to pipe_11_temperature:
    T_node[18] = (2.385662e+00 * P_node[17]) + (9.103613e-01 * T_node[17]) + (3.415186e-05 * Q_pipe[11]) + (3.659100e+01 * d) + (-3.246226e-05 * L) + (2.051846e-02 * delta_h) + (-37.646961);

#category 3 constraints
subject to safety_envelope_lower {m in 1..6}: Q_comp[m] >= Q_min[m];
subject to safety_envelope_upper {m in 1..5}: Q_comp[m] <= Q_max[m];
"""
)

print("Solving pipeline system operations optimization with Gurobi...")
ampl.solve(solver='gurobi')
print("-"*30)

print(f"Solver Status Summary : {ampl.solve_result}")
obj_val = ampl.get_objective("total_daily_energy_consumption").value()
print(f"Minimized Energy Consumption : {obj_val:.2f} tce/d\n")