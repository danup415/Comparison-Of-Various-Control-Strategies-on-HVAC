# Pune HVAC Simulink Project

A MATLAB/Simulink model of a three-zone Pune residential HVAC system with baseline, PID, MPC, and DQN controllers.

## Requirements

- MATLAB R2021a or later (the source header requirement); the project has been run in MATLAB R2024a.
- Simulink. The model uses MATLAB Function blocks; Stateflow support may be required for the installed MATLAB release and license.

## Main workflow

1. Open this repository folder as MATLAB's current folder.
2. Run `Pune_HVAC_Full_Simulink500hrs`.
3. The default mode creates synthetic demo weather/building inputs, trains the DQN controller, runs the MATLAB-side annual controller comparison, builds the Simulink model, and writes figures/results.

The script trains for 500 episodes and may take several minutes. It clears the MATLAB workspace and closes open figures. Building deletes and recreates `Pune_HVAC_Complete.slx`; the script overwrites generated files with matching names in the current folder. Use a working copy if you need to preserve existing outputs.

Although `RUN_SIMULATION` is displayed as true, the executable build path does not call `sim`. The MATLAB-side controller comparison and model generation run; the saved model must be simulated separately. The KPIs printed by the script are not results from a Simulink simulation.

## Opening the saved model

For the saved Complete model, keep these two files together at the repository root:

- `Pune_HVAC_Complete.slx`
- `SimWS_Full.mat`

The model workspace references `SimWS_Full` by name. The builder writes that MAT file in the current folder. The build log reported model-workspace changes that are not automatically saved, so reopening and simulating the saved model should be validated before treating it as independently runnable.

`postsimulation.m` is a separate post-processing/simulation path. It can run the model and produce further results, so it has simulation side effects.

## Repository layout

The runtime files remain together at the root because the builder and model use current-folder-relative filenames. `docs/FILE_GUIDE.md` describes the other included files and whether they are runtime inputs or retained artifacts.

The `.gitignore` excludes MATLAB editor/build/cache files only. It deliberately keeps the `.slx`, MAT files, weather file, and existing plots trackable.


