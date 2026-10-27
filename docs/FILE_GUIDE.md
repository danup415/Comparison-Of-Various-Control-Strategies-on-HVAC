# File guide

This guide describes each file by its likely role in the current `Pune_HVAC_Complete` workflow. “Optional” means not required by that path; it is not a deletion recommendation.

| File | Role | Status for saved Complete model |
|---|---|---|
| `Pune_HVAC_Full_Simulink500hrs.m` | Main synthetic-data/training/MATLAB annual-comparison and model-builder script | Needed to rebuild; not needed to open an existing model |
| `Pune_HVAC_Complete.slx` | Saved Simulink model | Required |
| `SimWS_Full.mat` | Model workspace time-series data | Required with the saved model |
| `postsimulation.m` | Separate Simulink post-processing and simulation script | Optional; executes the model and creates more outputs |
| `sim800hrs.m` | Alternate full builder/simulation implementation; can target the same model/output names | Optional alternative; do not treat as a second canonical entry point |
| `All_Results.mat` | MATLAB-side controller comparison results | Existing result artifact; not a model input |
| `RL_Network_Weights.mat` | Trained DQN weights saved by the builder | Training artifact; the current model embeds its required controller code/weights |
| `Fig01_ZoneTemperatures.png` | Zone temperature plot | Result image |
| `Fig2_TrackingError.png` | Existing tracking-error plot; the builder writes `Fig02_TrackingError.png` (zero after `Fig`) | Older/alternate result image |
| `Fig03_MonthlyEnergy.png` | Monthly energy plot | Result image |
| `Fig04_ComfortDistribution.png` | Comfort distribution plot | Result image |
| `Fig05_ToDLoadShifting.png` | Time-of-day load-shifting plot | Result image |
| `Fig06_IAQModel.png` | IAQ plot | Result image |
| `Fig07_RLTraining.png` | RL training plot | Result image |
| `Fig08_KPISummary.png` | KPI summary plot | Result image |
| `IND_Pune.430630_ISHRAE.epw` | Raw Pune weather file | Not used by the default synthetic-data path; no EPW-processing workflow is included here |
| `Pune_HVAC_DT_WS.mat` | Workspace for a separate Digital Twin model path | Not required; no corresponding DT SLX is included in this folder |
| `ctrl_baseline.m` | Standalone baseline controller | Reference function; not statically required by the saved Complete model |
| `ctrl_pid.m` | Standalone PID controller | Reference function; not statically required by the saved Complete model |
| `ctrl_mpc.m` | Standalone MPC controller | Reference function; not statically required by the saved Complete model |
| `ctrl_mux.m` | Standalone controller selector | Reference function; not statically required by the saved Complete model |
| `ctrl_rl_dqn.m` | Standalone fixed-weight DQN policy implementation | Reference function; not statically required by the saved Complete model |
| `hvac_equip.m` | Standalone HVAC equipment calculation | Reference function; not statically required by the saved Complete model |
| `hvac_equip_vec.m` | Vector-form HVAC equipment calculation | Reference function; not statically required by the saved Complete model |
| `iaq_model.m` | Standalone IAQ calculation | Reference function; not statically required by the saved Complete model |
| `iaq_model_vec.m` | Vector-form IAQ calculation | Reference function; not statically required by the saved Complete model |

## Generated on a full builder run

The main script writes or replaces `Pune_HVAC_Complete.slx`, `SimWS_Full.mat`, `RL_Network_Weights.mat`, `All_Results.mat`, and publication PNGs in the current folder. Keep the model and `SimWS_Full.mat` paired. The other results are useful for reporting/review but are not required to open the saved model.
