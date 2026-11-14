# What changed from the original project, and why

Every item below was a finding of the diagnostic audit (13 Nov 2026). The left column is
the original behaviour (file/function), the right column is the fix and where it lives now.

## Data
| Original | Fix |
|---|---|
| `generate_demo_data` produced synthetic sinusoidal weather; the ISHRAE EPW was never read | `src/weather/read_epw.m` reads `data/IND_Pune.430630_ISHRAE.epw` (8760 h, Tmax 41.0 C at ~14:00). No synthetic weather anywhere. |
| Synthetic diurnal phase peaked at 20:00 (6 h late) | Real EPW data; test 9 in `tests/run_tests.m` checks peak value and hour. |
| `datetime(2024,...)` leap-year calendar | EPW calendar; day-of-week from the EPW data period (starts Sunday). |
| Solar gain = `0.6*GHI*orient_f` | `solar_geometry.m` + `build_inputs.m`: sun position (Spencer), beam/diffuse/ground-reflected irradiance on each N/E/S/W facade, monthly chajja shading of beam only. |

## Building physics
| Original | Fix |
|---|---|
| One node per zone, C from 5 mm "stone" -> tau 2.2-2.6 h; paper reported 56-66 h | Two nodes per zone (air + thermal mass), `build_model.m`. Mass from RCC roof/floor slabs and AAC walls (Cm = 7-15 MJ/K); modal time constants 0.35-0.43 h (air) and 25-36 h (mass). |
| Explicit Euler, dt = 3600 s, clamp `max(8, ...)` produced the "8 C for 2000 h" artifact | Exact zero-order-hold discretisation (matrix exponential), dt = 900 s; no clamps. Test 1: agrees with RK4 to 1e-12 K. |
| U-values 0.163 (wall) / 0.268 (roof): super-insulated, untypical | 200 mm AAC wall U = 0.75, ENS-compliant roof U = 1.2 (stated as assumptions in `default_config.m`). |
| Cooking 2 kW all into Zone 2 | Kitchen unconditioned; 30 % of cooking sensible/latent leaks into Zone 2. |

## Equipment
| Original | Fix |
|---|---|
| Two different COP formulas (energy vs Fig. 5) | One VRF model (`vrf_model.m`): capacity derating, EIR(T), inverter part-load curve, cycling losses below 20 % PLR, sensible/latent split, indoor-fan power. |
| Cooling delivered with zero power outside 15-48 C (free cooling) | Outside the operating range the unit delivers nothing and draws nothing (test 6). |
| IEC capacity referenced to outdoor, 80 % effectiveness unused | Supply T = Tdb - 0.8 (Tdb - Twb); sensible effect m*cp*(Tsup - Tzone); only enabled when Tsup < Tzone - 2 K; its air counts for CO2/PM/moisture. |
| DOAS had no thermal effect on zones; weather-triggered | DOAS supply (after ERV and demand-controlled dehumidifying coil) enters the zone heat and moisture balances; continuous ASHRAE 62.2 rate (28 L/s); original humidity-triggered mode kept as a scenario. |

## IAQ
| Original | Fix |
|---|---|
| PM2.5 explicit Euler with k*dt = 4.5 -> oscillation 0 / 22 / 172 ug/m3 ("160-195 ug/m3" headline) | Exact exponential update of linear mass balances (test 4: monotone even for k*dt = 40). |
| No exfiltration term | Outflow = infiltration + DOAS + IEC airflow. |
| Filtration conclusions based on artifact | IAQ scenarios: no filter / HEPA recirculation / humidity-triggered DOAS / no ventilation. |

## Controllers
| Original | Fix |
|---|---|
| Baseline cooled regardless of indoor temperature (and used setback, contrary to paper) | Per-zone on/off thermostat with hysteresis (realistic split-AC baseline). |
| Setpoint asymmetry (Baseline/RL fixed 24 C, others setback) | One occupancy-based setpoint/band schedule (`D.sp`, `D.Tlo`, `D.Thi`) for every controller. |
| PID gains in paper != code; one-sided error; unit confusion | SIMC tuning from the identified model (`tune_simc.m`), back-calculation anti-windup, filtered derivative. |
| "MPC" had no input in its prediction, no cost, no optimisation; tariff term disconnected | True economic MPC (`mpc_core_*.m`): 6-state model, 24 h horizon, ToD-cost objective, soft comfort constraints with back-off, input/capacity constraints, FISTA solver with exact projection, offset-free Kalman filter, noisy weather/solar forecast. |
| Hybrid controller existed only in the paper | Implemented (`ctrl_hybrid.m`): E-MPC feed-forward + PI correction on the MPC prediction + safety override. |
| DQN: trained and tested on the same synthetic year, best checkpoint chosen from exploration returns, duplicate feature, single seed, IEC train/test mismatch | Parameter-shared double DQN (`rl_train.m`): training on perturbed weather, checkpoint chosen on a separate greedy validation set, 15 observable features incl. setpoint & occupancy, Huber loss, Adam, gradient clipping, target network, 5 seeds; same plant (incl. IEC) for training and test. |
| Plant = controller model | Plant/model mismatch (C, UA, solar, internal gains) + sensor noise + forecast error in the main scenario; perfect-information MPC as an ablation bound. |

## Evaluation
| Original | Fix |
|---|---|
| Cost = energy x flat 7.15 (claimed ToD) | Cost = sum of 15-min power x hourly ToD tariff; flat-tariff scenario separately. |
| Fig. 3 monthly energy = annual energy pro-rated by days | Real monthly sums. |
| Fig. 5 hourly VRF power = constant line by construction | Real mean hourly profile (cooling season) vs tariff. |
| Zone-2-only comfort incl. unoccupied hours | All zones, occupied hours; overheating + cooling-induced overcooling (cold due to weather, with no heating installed, separated using a free-floating reference run). |
| "Statistically indistinguishable" with one run | 5 seeds (sensor noise, forecast noise, DQN training) with 95 % t-CIs; paired daily block bootstrap + Wilcoxon tests. |
| Simulink post-processing summed variable-step samples as kWh | Fixed-step 900 s engine; Simulink wrapper calls the same code (`simulink/`), with an equivalence check. |

## Reproducibility
`run_all.m` regenerates every table and figure; all parameters in `config/default_config.m`;
`tests/run_tests.m` (19 checks); runs on MATLAB (no toolboxes) and GNU Octave.

## Verification round (Nov 2026)
- printf -> fprintf and -v7 .mat saves for MATLAB portability.
- Grid factor 0.82 -> 0.727 (CEA v20, FY2023-24); three-tier tariff relabelled hypothetical; NBC 2016 -> NBC 2005.
- New scenarios: heat_perfect, heat_backoff1, tariff_steep, tariff_msedcl, hyb_noPI, hyb_nosafety, uq1..uq40; new analyze_uq.m, check_tariff_mechanism.m, validation/energyplus/.
- Manuscript: heatwave explanation changed from forecast error to margin/back-off; Hybrid explanation split into PI (cost) and safety override (comfort); tariff discussion reworded and extended; UQ results; tuning-on-evaluation-year disclosure.
