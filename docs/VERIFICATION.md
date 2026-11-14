# Verification record

This file records what was checked after the rebuild, how each item was checked, and what is still open.

## 1. Code runs in MATLAB and Simulink (VERIFIED)

| Check | Result |
|---|---|
| `tests/run_tests.m` on MATLAB R2024a (Windows) | 19 of 19 tests pass |
| Noise-free 14-day run, MATLAB vs Octave 10.3 | Identical to about 1e-12 |
| Annual KPIs, MATLAB vs Octave (same seed) | Differ by 0.5 % or less, because the platforms' random-number streams differ |
| Simulink wrapper (`simulink/check_simulink_equivalence.m`), 7 days, PID and Hybrid | max abs(dTa) = 0, max abs(dP) = 0 over 672 samples |
| Ideal sensible cooling loads (RC model) | [3300 7365 2355] kWh, identical in MATLAB and Octave |

Two portability fixes came out of these checks:
- `printf` was replaced by `fprintf`.
- All `.mat` files are now saved with `-v7`.

## 2. Explanations in the paper (TESTED; two revised)

| Claim | Test (scenario) | Outcome |
|---|---|---|
| In the heatwave, E-MPC loses to PID because of forecast or model error | `heat_perfect`: E-MPC with perfect model and weather | **Rejected.** Kh = 142, no better than 126. |
| E-MPC loses because it rides the upper bound with too little margin | `heat_backoff1`: back-off 0.5 to 1.0 K | **Supported.** Kh falls from 126 to 73 (PID 65); cost rises to INR 53,030. |
| Hybrid beats E-MPC because PI corrects intra-hour error | `hyb_noPI`, `hyb_nosafety` | **Partly.** The PI layer gives the cost gain; about 70 % of the comfort gain comes from the safety override. Text revised. |
| Little ToD shifting because the evening peak coincides with occupancy | `tests/check_tariff_mechanism.m` | **Reworded.** 100 % of peak-window VRF energy serves occupied zones, but occupied zone-hours fall in peak windows less often than hours overall (27 % vs 33 %). |
| ...because slab heat is released in the evening | same | **Supported.** Slab-to-air flow is about 2.5 kW at 18–23 h vs 1.8 kW at 10–16 h. |
| ...because the 1.7:1 ratio is modest | `tariff_steep` (3.6:1) | **Supported.** The E-MPC/Hybrid peak share falls from 0.34 to 0.29–0.30. |

## 3. Assumed inputs (TESTED for robustness, not for truth)

`uq1..uq40` is a Latin hypercube over 11 assumed inputs (`src/sim/uq_config.m`), analysed with `analyze_uq`. Results are in `results/tables/uq_summary.csv`.
- Hybrid has the lowest discomfort among the modulating controllers in 100 % of samples, and the lowest cost in 62.5 % (DQN in the rest).
- Hybrid beats PID on both cost and discomfort in 95 % of samples, and beats E-MPC in 100 %.
- Hybrid-vs-PID cost difference: 5th / 50th / 95th percentile = -6.5 / -4.3 / -0.2 %.
- The saving shrinks with higher roof U-value (rho = +0.60) and higher SHGC (+0.49). It grows with thermal mass (-0.38).

The ranking is robust; the size of the saving is not. The inputs themselves remain assumptions.

## 4. Facts and references (CHECKED; three corrected)

**Corrections:**
- Grid factor: 0.82 was unsupported. CEA CO2 Baseline Database v20 gives 0.727 kgCO2/kWh for FY2023-24, and India has one national grid factor.
- Tariff: the three-tier tariff is **hypothetical**, not MSEDCL's. The actual MSEDCL residential structure (a INR 0.80 rebate from 09:00 to 17:00) was added as `tariff_msedcl`. Under it, Hybrid is most comfortable and 4.4 % cheaper than PID; DQN is 0.1 % cheaper than Hybrid.
- Comfort band: 23–26 C comes from NBC 2005; NBC 2016 uses an adaptive model.

**Confirmed:**
- All other cited papers: authors, venue, year and pages.
- IEA 2023 figures.
- ECBC/ENS roof U-value of 1.2 or less.
- The ASHRAE 62.2 formula.

## 5. Still open (needs the author)

1. **EnergyPlus cross-check.**
   - Install EnergyPlus 24.2.
   - In MATLAB, run `make_energyplus_case; run_energyplus_comparison('C:\EnergyPlusV24-2-0\energyplus.exe')`.
   - The case files are ready. EnergyPlus could not be downloaded in the build environment, so this has not been run.
2. **MERC order text.** RESOLVED. The MYT order of 28 Mar 2025 (Case 217 of 2024) and the review order of 25 Jun 2025 give a residential ToD rebate of INR 0.80/kWh during solar hours (09:00-17:00) in FY 2025-26, rising to INR 1.00/kWh by FY 2029-30; the night rebate was removed. The `tariff_msedcl` scenario models the FY 2025-26 rebate on a single average energy rate of INR 7.15/kWh, a simplification of the real slab tariff.
3. **NBC 2005 clause.** RESOLVED. NBC 2005, Part 8, Section 3, Cl. 4.4.3 (Inside design conditions), Table 2 gives 23-26 C DB for summer in air-conditioned occupancies. The table has no residential category; the paper applies this range to the dwelling.
4. **Measured PM2.5.** Replace the representative outdoor profile with CPCB CAAQMS hourly data for Pune.
5. **New scenarios are single-seed.** The new scenarios (heat_*, tariff_*, hyb_*, uq*) are single-seed MATLAB runs; the main study uses 5 seeds.
