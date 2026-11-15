function mdl = build_pune_simulink_model(n_days)
%BUILD_PUNE_SIMULINK_MODEL  Build Pune_HVAC_BlockDiagram.slx, a block-diagram Simulink model
%   of the full study: weather and disturbances, sensors, five selectable controllers,
%   and a plant made of VRF, IEC, DOAS, building (RC) and indoor-air-quality subsystems.
%
%   Native Simulink blocks: the building thermal model (Discrete State-Space with the
%   exact zero-order-hold matrices), weather/disturbance and noise inputs (From
%   Workspace), sensor sums, gains, unit delays, the controller selector and all
%   signal routing. Nonlinear parts are MATLAB Function blocks that call sl_blocks.m.
%
%   Usage (from the project root, after setup_paths):
%     SL = sl_blocks('init', 'base', 'hybrid', 1, 7); assignin('base', 'SL', SL);
%     mdl = build_pune_simulink_model(7);  out = sim(mdl);
%   Controller choice: SL.CTRL = 1 thermostat, 2 PID, 3 E-MPC, 4 DQN, 5 Hybrid.
%   Requires MATLAB + Simulink (R2020b or later).
if nargin < 1, n_days = 7; end
mdl = 'Pune_HVAC_BlockDiagram';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
Ts = 900;
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', 'SL.Ts', ...
    'StopTime', sprintf('(%d*96-1)*SL.Ts', n_days), 'SaveOutput', 'off', ...
    'ReturnWorkspaceOutputs', 'on', 'AlgebraicLoopMsg', 'error');

C.input = '[0.85 0.93 1.0]'; C.ctrl = '[1.0 0.92 0.80]'; C.plant = '[0.85 1.0 0.88]';
C.sense = '[0.95 0.88 1.0]';  C.log = '[0.92 0.92 0.92]';

%% ---------------- top level ----------------
blk(mdl, 'simulink/Sources/Counter Free-Running', 'Step counter', [30 40 100 70], ...
    'NumBits', '32', 'tsamp', 'SL.Ts');
blk(mdl, 'simulink/Signal Attributes/Data Type Conversion', 'to double', [130 40 190 70], ...
    'OutDataTypeStr', 'double');
blk(mdl, 'simulink/Math Operations/Bias', 'k = n + 1', [220 40 270 70], 'Bias', '1');
blk(mdl, 'simulink/Sources/From Workspace', 'Weather and disturbances', [30 150 200 200], ...
    'VariableName', 'SL.DIST', 'SampleTime', 'SL.Ts', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'BackgroundColor', C.input);
blk(mdl, 'simulink/Sources/From Workspace', 'Sensor noise', [30 260 200 300], ...
    'VariableName', 'SL.NOISE', 'SampleTime', 'SL.Ts', 'Interpolate', 'off', ...
    'OutputAfterFinalValue', 'Holding final value', 'BackgroundColor', C.input);
add_line(mdl, 'Step counter/1', 'to double/1');
add_line(mdl, 'to double/1', 'k = n + 1/1');

% Sensors
S = [mdl '/Sensors'];
add_block('simulink/Ports & Subsystems/Subsystem', S, 'Position', [300 240 400 320], ...
    'BackgroundColor', C.sense);
clear_subsystem(S);
port(S, 'In', 'Ta (true)', 1, [30 40 60 54]);
port(S, 'In', 'noise', 2, [30 100 60 114]);
blk(S, 'simulink/Math Operations/Add', 'Add noise', [120 60 150 100]);
port(S, 'Out', 'Tmeas', 1, [210 73 240 87]);
add_line(S, 'Ta (true)/1', 'Add noise/1'); add_line(S, 'noise/1', 'Add noise/2');
add_line(S, 'Add noise/1', 'Tmeas/1');

% Controllers
build_controllers([mdl '/Controllers'], [480 30 640 330], C.ctrl);

% Plant
build_plant([mdl '/Plant'], [740 30 930 400], C.plant);

% wiring
add_line(mdl, 'Sensor noise/1', 'Sensors/2');
add_line(mdl, 'k = n + 1/1', 'Controllers/1', 'autorouting', 'smart');
add_line(mdl, 'Sensors/1', 'Controllers/2', 'autorouting', 'smart');
add_line(mdl, 'Controllers/1', 'Plant/1', 'autorouting', 'smart');
add_line(mdl, 'Weather and disturbances/1', 'Plant/2', 'autorouting', 'smart');
add_line(mdl, 'Plant/1', 'Sensors/1', 'autorouting', 'smart');

% logging and scopes
logs = {'Ta', 1; 'Tm', 2; 'P', 3; 'W', 4; 'CO2', 5; 'PM', 6};
for i = 1:size(logs, 1)
    y = 30 + 70*(i-1);
    blk(mdl, 'simulink/Sinks/To Workspace', ['log ' logs{i,1}], [1080 y 1160 y+30], ...
        'VariableName', ['sl_' logs{i,1}], 'SaveFormat', 'Array', 'SampleTime', 'SL.Ts', ...
        'BackgroundColor', C.log);
    add_line(mdl, sprintf('Plant/%d', logs{i,2}), ['log ' logs{i,1} '/1'], 'autorouting', 'smart');
end
blk(mdl, 'simulink/Sinks/To Workspace', 'log u', [740 450 820 480], 'VariableName', 'sl_u', ...
    'SaveFormat', 'Array', 'SampleTime', 'SL.Ts', 'BackgroundColor', C.log);
add_line(mdl, 'Controllers/1', 'log u/1', 'autorouting', 'smart');
blk(mdl, 'simulink/Sinks/Scope', 'Zone temperatures', [1220 30 1260 70]);
blk(mdl, 'simulink/Sinks/Scope', 'Electric power', [1220 170 1260 210]);
blk(mdl, 'simulink/Sinks/Scope', 'Indoor air quality', [1220 310 1260 350], 'NumInputPorts', '3');
add_line(mdl, 'Plant/1', 'Zone temperatures/1', 'autorouting', 'smart');
add_line(mdl, 'Plant/3', 'Electric power/1', 'autorouting', 'smart');
for i = 1:3, add_line(mdl, sprintf('Plant/%d', 3+i), sprintf('Indoor air quality/%d', i), 'autorouting', 'smart'); end

add_annotation(mdl, [30 420 640 520], sprintf([ ...
    'Pune 3BHK apartment: cooling-control benchmark (15-min step)\n' ...
    'Select the controller with SL.CTRL: 1 thermostat, 2 PID, 3 E-MPC, 4 DQN, 5 Hybrid.\n' ...
    'Initialise with SL = sl_blocks(''init'', scenario, controller, seed, n_days).']));
save_system(mdl);
end

%% ======================================================================
function build_controllers(sys, pos, color)
add_block('simulink/Ports & Subsystems/Subsystem', sys, 'Position', pos, 'BackgroundColor', color);
clear_subsystem(sys);
port(sys, 'In', 'k', 1, [30 40 60 54]);
port(sys, 'In', 'Tmeas', 2, [30 90 60 104]);
names = {'Thermostat', 'PID (SIMC)', 'E-MPC', 'DQN (shielded)', 'Hybrid'};
keys  = {'thermostat', 'pid', 'mpc', 'rl', 'hybrid'};
blk(sys, 'simulink/Sources/Constant', 'SL.CTRL', [140 470 200 500], 'Value', 'SL.CTRL');
blk(sys, 'simulink/Signal Routing/Multiport Switch', 'Controller selector', [560 120 600 400], ...
    'Inputs', '5', 'DataPortOrder', 'One-based contiguous');
add_line(sys, 'SL.CTRL/1', 'Controller selector/1', 'autorouting', 'smart');
for i = 1:5
    y = 40 + 85*(i-1); E = [sys '/' names{i}];
    add_block('simulink/Ports & Subsystems/Enabled Subsystem', E, 'Position', [300 y 440 y+60]);
    clear_subsystem(E, true);
    port(E, 'In', 'k', 1, [30 40 60 54]); port(E, 'In', 'Tmeas', 2, [30 90 60 104]);
    mfun(E, 'control law', [120 40 260 110], sprintf([ ...
        'function u = control_law(k, Tmeas)\n' ...
        '%% %s controller (see src/controllers)\n' ...
        'coder.extrinsic(''sl_blocks'');\n' ...
        'u = zeros(3,1);\n' ...
        'u = sl_blocks(''ctrl'', ''%s'', k, Tmeas);\n'], names{i}, keys{i}));
    port(E, 'Out', 'u', 1, [320 68 350 82]);
    add_line(E, 'k/1', 'control law/1'); add_line(E, 'Tmeas/1', 'control law/2');
    add_line(E, 'control law/1', 'u/1');
    blk(sys, 'simulink/Logic and Bit Operations/Compare To Constant', ['is ' keys{i}], ...
        [140 y+70 200 y+80], 'const', num2str(i), 'relop', '==');
    add_line(sys, 'SL.CTRL/1', ['is ' keys{i} '/1'], 'autorouting', 'smart');
    add_line(sys, ['is ' keys{i} '/1'], [names{i} '/Enable'], 'autorouting', 'smart');
    add_line(sys, 'k/1', [names{i} '/1'], 'autorouting', 'smart');
    add_line(sys, 'Tmeas/1', [names{i} '/2'], 'autorouting', 'smart');
    add_line(sys, [names{i} '/1'], sprintf('Controller selector/%d', i+1), 'autorouting', 'smart');
end
port(sys, 'Out', 'Qcmd', 1, [660 253 690 267]);
add_line(sys, 'Controller selector/1', 'Qcmd/1');
end

%% ======================================================================
function build_plant(sys, pos, color)
add_block('simulink/Ports & Subsystems/Subsystem', sys, 'Position', pos, 'BackgroundColor', color);
clear_subsystem(sys);
port(sys, 'In', 'Qcmd', 1, [20 100 50 114]);
port(sys, 'In', 'dist', 2, [20 400 50 414]);
% disturbance channels
sel(sys, 'To',        '1',        [110 300 150 320]);
sel(sys, 'Wo',        '2',        [110 340 150 360]);
sel(sys, 'Twb',       '3',        [110 380 150 400]);
sel(sys, 'RHout',     '4',        [110 420 150 440]);
sel(sys, 'To, sol-air', '[1 5 6 7 8]', [110 470 150 490]);
sel(sys, 'Qsol',      '[9 10 11]',  [110 520 150 540]);
sel(sys, 'Qint',      '[12 13 14]', [110 570 150 590]);
for nm = {'To','Wo','Twb','RHout','To, sol-air','Qsol','Qint'}
    add_line(sys, 'dist/1', [nm{1} '/1'], 'autorouting', 'smart');
end
% state memories for humidity, CO2, PM2.5
blk(sys, 'simulink/Discrete/Unit Delay', 'W(k)',   [300 660 340 690], 'InitialCondition', 'SL.W0',   'SampleTime', 'SL.Ts');
blk(sys, 'simulink/Discrete/Unit Delay', 'CO2(k)', [300 720 340 750], 'InitialCondition', 'SL.CO2_0', 'SampleTime', 'SL.Ts');
blk(sys, 'simulink/Discrete/Unit Delay', 'PM(k)',  [300 780 340 810], 'InitialCondition', 'SL.PM0',  'SampleTime', 'SL.Ts');

mfun(sys, 'VRF (inverter, part load, latent)', [420 60 600 160], sprintf([ ...
    'function [Qs, mw, P_vrf, n_on] = vrf(Qcmd, To, W)\n' ...
    'coder.extrinsic(''sl_blocks'');\n' ...
    'Qs = zeros(3,1); mw = 0; P_vrf = 0; n_on = 0;\n' ...
    '[Qs, mw, P_vrf, n_on] = sl_blocks(''vrf'', Qcmd, To, W);\n']));
mfun(sys, 'IEC (indirect evaporative)', [420 200 600 300], sprintf([ ...
    'function [Q_iec, P_iec, F_iec] = iec(Qcmd, Ta, To, Twb, RHout)\n' ...
    'coder.extrinsic(''sl_blocks'');\n' ...
    'Q_iec = zeros(3,1); P_iec = 0; F_iec = 0;\n' ...
    '[Q_iec, P_iec, F_iec] = sl_blocks(''iec'', Qcmd, Ta, To, Twb, RHout);\n']));
mfun(sys, 'DOAS + ERV + coil', [420 340 600 440], sprintf([ ...
    'function [Q_doas, P_doas, F_d, W_d] = doas(Ta, W, To, Wo, RHout)\n' ...
    'coder.extrinsic(''sl_blocks'');\n' ...
    'Q_doas = zeros(3,1); P_doas = 0; F_d = 0; W_d = 0;\n' ...
    '[Q_doas, P_doas, F_d, W_d] = sl_blocks(''doas'', Ta, W, To, Wo, RHout);\n']));
mfun(sys, 'Moisture, CO2, PM2.5 balances', [420 650 600 820], sprintf([ ...
    'function [W1, CO2_1, PM1] = iaq(W, CO2, PM, mw, F_d, W_d, F_iec, dist)\n' ...
    'coder.extrinsic(''sl_blocks'');\n' ...
    'W1 = 0; CO2_1 = 0; PM1 = 0;\n' ...
    '[W1, CO2_1, PM1] = sl_blocks(''iaq'', W, CO2, PM, mw, F_d, W_d, F_iec, dist);\n']));

blk(sys, 'simulink/Math Operations/Gain', 'solar gain factor',    [700 520 760 540], 'Gain', 'SL.gs');
blk(sys, 'simulink/Math Operations/Gain', 'internal gain factor', [700 570 760 590], 'Gain', 'SL.gi');
blk(sys, 'simulink/Math Operations/Add', 'ventilation heat', [700 300 730 340]);
blk(sys, 'simulink/Signal Routing/Mux', 'Mux', [820 100 825 600], 'Inputs', '5');
blk(sys, 'simulink/Discrete/Discrete State-Space', 'Building RC model (exact ZOH)', [880 300 1060 400], ...
    'A', 'SL.Ad', 'B', 'SL.B', 'C', 'eye(6)', 'D', 'zeros(6,17)', ...
    'InitialCondition', 'SL.x0', 'SampleTime', 'SL.Ts');
sel(sys, 'air nodes', '1:3', [1120 320 1160 340]); sel(sys, 'mass nodes', '4:6', [1120 370 1160 390]);
blk(sys, 'simulink/Sources/Constant', 'filter fan', [700 880 760 900], 'Value', 'SL.P_filt');
blk(sys, 'simulink/Math Operations/Sum', 'total power', [880 820 910 900], 'Inputs', '++++');

% outputs: 1 Ta, 2 Tm, 3 P, 4 W, 5 CO2, 6 PM
outs = {'Ta','Tm','P','W','CO2','PM'};
for i = 1:6, port(sys, 'Out', outs{i}, i, [1240 40+60*i 1270 54+60*i]); end

V = 'VRF (inverter, part load, latent)'; I = 'IEC (indirect evaporative)';
Dd = 'DOAS + ERV + coil'; Q = 'Moisture, CO2, PM2.5 balances'; B = 'Building RC model (exact ZOH)';
L = @(a, b) add_line(sys, a, b, 'autorouting', 'smart');
% VRF
L('Qcmd/1', [V '/1']); L('To/1', [V '/2']); L('W(k)/1', [V '/3']);
% IEC
L('Qcmd/1', [I '/1']); L('air nodes/1', [I '/2']); L('To/1', [I '/3']); L('Twb/1', [I '/4']); L('RHout/1', [I '/5']);
% DOAS
L('air nodes/1', [Dd '/1']); L('W(k)/1', [Dd '/2']); L('To/1', [Dd '/3']); L('Wo/1', [Dd '/4']); L('RHout/1', [Dd '/5']);
% building inputs: [Qs; To, Tsa_roof, Tsa_wall; gs*Qsol; gi*Qint; Q_iec + Q_doas]
L([V '/1'], 'Mux/1'); L('To, sol-air/1', 'Mux/2');
L('Qsol/1', 'solar gain factor/1'); L('solar gain factor/1', 'Mux/3');
L('Qint/1', 'internal gain factor/1'); L('internal gain factor/1', 'Mux/4');
L([I '/1'], 'ventilation heat/1'); L([Dd '/1'], 'ventilation heat/2'); L('ventilation heat/1', 'Mux/5');
L('Mux/1', [B '/1']); L([B '/1'], 'air nodes/1'); L([B '/1'], 'mass nodes/1');
% moisture / IAQ
L('W(k)/1', [Q '/1']); L('CO2(k)/1', [Q '/2']); L('PM(k)/1', [Q '/3']);
L([V '/2'], [Q '/4']); L([Dd '/3'], [Q '/5']); L([Dd '/4'], [Q '/6']); L([I '/3'], [Q '/7']);
L('dist/1', [Q '/8']);
L([Q '/1'], 'W(k)/1'); L([Q '/2'], 'CO2(k)/1'); L([Q '/3'], 'PM(k)/1');
% power
L([V '/3'], 'total power/1'); L([I '/2'], 'total power/2'); L([Dd '/2'], 'total power/3');
L('filter fan/1', 'total power/4');
% outputs
L('air nodes/1', 'Ta/1'); L('mass nodes/1', 'Tm/1'); L('total power/1', 'P/1');
L([Q '/1'], 'W/1'); L([Q '/2'], 'CO2/1'); L([Q '/3'], 'PM/1');
% unused VRF output (compressor count) to a terminator
blk(sys, 'simulink/Sinks/Terminator', 'n_on', [660 140 680 160]); L([V '/4'], 'n_on/1');
end

%% ---------------- helpers ----------------
function blk(sys, lib, name, pos, varargin)
add_block(lib, [sys '/' name], 'Position', pos, varargin{:});
end

function sel(sys, name, idx, pos)
add_block('simulink/Signal Routing/Selector', [sys '/' name], 'Position', pos, ...
    'NumberOfDimensions', '1', 'IndexMode', 'One-based', 'IndexOptions', 'Index vector (dialog)', ...
    'Indices', idx, 'InputPortWidth', '-1');
end

function port(sys, kind, name, n, pos)
if strcmp(kind, 'In'), lib = 'simulink/Sources/In1'; else, lib = 'simulink/Sinks/Out1'; end
add_block(lib, [sys '/' name], 'Position', pos, 'Port', num2str(n));
end

function mfun(sys, name, pos, code)
path = [sys '/' name];
add_block('simulink/User-Defined Functions/MATLAB Function', path, 'Position', pos);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', path);
ch.Script = code;
end

function clear_subsystem(sys, keep_enable)
% remove the default In1 -> Out1 contents (keep the Enable port of enabled subsystems)
if nargin < 2, keep_enable = false; end
lines = find_system(sys, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line');
for h = lines', delete_line(h); end
b = find_system(sys, 'SearchDepth', 1, 'Type', 'block');
for i = 2:numel(b)
    if keep_enable && strcmp(get_param(b{i}, 'BlockType'), 'EnablePort'), continue; end
    delete_block(b{i});
end
end

function add_annotation(sys, pos, txt)
try
    a = Simulink.Annotation(sys, txt); a.Position = pos; a.FontSize = 11;
catch
    % annotations are cosmetic; ignore if this release uses a different API
end
end
