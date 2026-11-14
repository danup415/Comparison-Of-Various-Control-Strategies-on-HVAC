# Residential Cooling Control Benchmark for a Pune Apartment

A reproducible, year-long comparison of five cooling controllers on one simulation model
of a 95 m² top-floor 3BHK apartment in Pune, India:

- split-AC **thermostat** (on/off)
- **PID** (SIMC-tuned)
- **economic MPC** (E-MPC)
- **shielded double DQN** (deep reinforcement learning)
- **Hybrid**: E-MPC + PI correction + safety override

The plant includes an inverter VRF, an indirect evaporative cooler (IEC), a dedicated
outdoor-air system (DOAS) with energy recovery, and humidity, CO₂ and PM2.5 models. It
is simulated over the ISHRAE typical year at a 15-min step. All controllers face
plant–model mismatch, sensor noise and forecast error, and the main study is repeated
over five seeds.

The code is plain MATLAB (no toolboxes needed) and also runs in GNU Octave ≥ 8. Every
number in the papers is generated from `results/tables/`.

## Headline results (full year, mean ± 95 % CI over 5 seeds)

| Controller | Cost (INR/yr) | Energy (kWh) | Discomfort (K h) | Acceptable occupied hours | Compressor starts |
|---|---:|---:|---:|---:|---:|
| Thermostat | 36,950 ± 8 | 5,028 | 1,102 ± 3 | 82.7 % | 8,476 |
| PID (SIMC) | 39,460 ± 3 | 5,403 | 315 ± 0.5 | 94.8 % | 488 |
| E-MPC | 38,306 ± 26 | 5,243 | 251 ± 2 | 95.7 % | 337 |
| DQN (shielded) | 38,799 ± 682 | 5,273 | 259 ± 54 | 94.3 % | 4,897 |
| Hybrid | 38,005 ± 14 | 5,198 | 185 ± 1 | 96.1 % | 750 |

- **Hybrid vs PID:** the Hybrid is 3.7 % cheaper with 41 % less discomfort (paired
  tests, p < 0.001).
- **Thermostat:** it is cheapest only because it overcools and short-cycles.
- **Robustness:** across a 40-sample variation of eleven assumed building inputs, the
  Hybrid had the lowest discomfort in every sample. It beat PID on both cost and
  discomfort in 95 % of the samples.

Full tables are in `results/summary.md` and `results/tables/`.

## Quick start

```matlab
>> setup_paths          % run from the repository root
>> run_tests            % 19 numerical and behavioural checks (about 1 min)
>> run_all              % full study: about 5 CPU-hours; writes results/runs/
```

To run in parallel, start two sessions with `PART='odd'; run_all` and
`PART='even'; run_all`, then run `PART='post'; run_all` to build tables and figures.

Trained DQN policies are included in `results/policies/`, so the DQN runs do not need to
retrain. The per-run output files (`results/runs/`, about 190 MB) are not stored in the
repository; `run_all` regenerates them.

## Repository layout

```
config/default_config.m      every model and controller parameter, tagged [ASSUMED]/[SOURCE]
data/                        ISHRAE EPW weather file for Pune (station 430630)
src/weather/                 EPW reader, solar geometry, psychrometrics
src/building/                2-node-per-zone RC model (exact zero-order hold), input series
src/plant/                   plant step (thermal, moisture, CO2, PM2.5) and VRF model
src/controllers/             thermostat, PID, E-MPC (FISTA + Kalman), DQN + shield, Hybrid
src/sim/                     simulation engine, scenarios, uncertainty samples, run_case
src/metrics/                 KPIs, paired statistics, aggregation, uncertainty analysis
src/viz/                     figure generation
simulink/                    Simulink wrapper around the same engine + equivalence check
tests/                       unit/regression tests, engine and tariff-mechanism checks
validation/energyplus/       EnergyPlus model of the same apartment + comparison script
results/                     tables (csv/tex), figures (png), summary.md, trained DQN policies
paper/ieee/                  paper in IEEE conference format (IEEEtran)
paper/springer/              paper in Springer LNCS format (llncs)
paper/preprint/              long-form preprint (article class) + numbers generator
docs/                        verification record and change log
```

## Building the papers

Each folder under `paper/` compiles on its own:

```
pdflatex main && bibtex main && pdflatex main && pdflatex main
```

`paper/ieee` and `paper/springer` can be uploaded to Overleaf as they are.

After re-running the simulations, refresh the numbers in all three papers with:

```
python3 paper/preprint/make_numbers.py
```

## Simulink

- `simulink/build_simulink_wrapper.m` builds a fixed-step (900 s) Simulink model whose
  MATLAB Function block calls the same engine (`sl_engine.m`).
- `check_simulink_equivalence.m` compares it with `simulate.m`. In MATLAB R2024a the
  outputs matched exactly over a 7-day test.

## EnergyPlus cross-check (not yet run)

Install EnergyPlus 24.2, then run:

```matlab
>> run_energyplus_comparison('C:\EnergyPlusV24-2-0\energyplus.exe')
```

The script:
- generates the EnergyPlus input files, using the same geometry, constructions, gains
  and weather as the RC model;
- runs EnergyPlus;
- reports bias, RMSE and R² for free-floating temperatures, and NMBE/CV(RMSE) for the
  ideal cooling loads.

## Verification status

`docs/VERIFICATION.md` lists:
- **verified:** MATLAB/Octave/Simulink equivalence, explanation tests, the uncertainty
  analysis and the references;
- **still open:**
  - the EnergyPlus cross-check;
  - measured outdoor PM2.5. The current profile is representative; replace it with CPCB
    CAAQMS hourly data for a Pune station.

## Citation

See `CITATION.cff`, or use the "Cite this repository" button on GitHub.

## License

No license has been chosen yet. Until one is added, all rights are reserved by the
author.
