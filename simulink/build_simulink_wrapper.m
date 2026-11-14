function mdl = build_simulink_wrapper(n_days)
%BUILD_SIMULINK_WRAPPER  Build Pune_HVAC_DT.slx: a fixed-step (900 s) discrete Simulink
%   model whose single MATLAB Function block calls the validated MATLAB engine
%   (sl_engine) every step. Use it to inspect signals in Simulink scopes or to
%   embed the study in a larger Simulink system. Requires MATLAB + Simulink.
%   (Not runnable in GNU Octave.)
%
%   Usage:
%     sl_engine('init','base','hybrid',1);
%     mdl = build_simulink_wrapper(7);  out = sim(mdl);
if nargin < 1, n_days = 365; end
mdl = 'Pune_HVAC_DT';
if bdIsLoaded(mdl), close_system(mdl, 0); end
new_system(mdl);
Ts = 900;
set_param(mdl, 'SolverType', 'Fixed-step', 'Solver', 'FixedStepDiscrete', 'FixedStep', num2str(Ts), ...
          'StopTime', num2str((n_days*96 - 1)*Ts), 'SaveOutput', 'on', 'SaveFormat', 'Dataset');
% step index k = t/Ts + 1 from a discrete counter
add_block('simulink/Sources/Counter Free-Running', [mdl '/k_counter'], 'NumBits', '32', ...
          'tsamp', num2str(Ts), 'Position', [30 80 90 110]);
add_block('simulink/Signal Attributes/Data Type Conversion', [mdl '/to_double'], ...
          'OutDataTypeStr', 'double', 'Position', [120 80 170 110]);
add_block('simulink/Math Operations/Bias', [mdl '/plus1'], 'Bias', '1', 'Position', [200 80 240 110]);
add_block('simulink/User-Defined Functions/MATLAB Function', [mdl '/HVAC_engine'], 'Position', [290 60 460 140]);
code = sprintf([ ...
 'function [Ta, P, u] = HVAC_engine(k)\n' ...
 '%%#codegen\n' ...
 'coder.extrinsic(''sl_engine'');\n' ...
 'Ta = zeros(3,1); P = 0; u = zeros(3,1);\n' ...
 '[Ta, P, u] = sl_engine(''step'', k);\n' ...
 'end\n']);
rt = sfroot; ch = rt.find('-isa', 'Stateflow.EMChart', 'Path', [mdl '/HVAC_engine']);
ch.Script = code;
names = {'Ta','P','u'};
for i = 1:3
    add_block('simulink/Sinks/To Workspace', [mdl '/log_' names{i}], 'VariableName', ['sl_' names{i}], ...
              'SaveFormat', 'Array', 'SampleTime', num2str(Ts), 'Position', [540 40+50*(i-1) 620 70+50*(i-1)]);
end
add_block('simulink/Sinks/Scope', [mdl '/Scope_Ta'], 'Position', [540 200 580 240]);
add_line(mdl, 'k_counter/1', 'to_double/1');
add_line(mdl, 'to_double/1', 'plus1/1');
add_line(mdl, 'plus1/1', 'HVAC_engine/1');
for i = 1:3, add_line(mdl, sprintf('HVAC_engine/%d', i), sprintf('log_%s/1', names{i})); end
add_line(mdl, 'HVAC_engine/1', 'Scope_Ta/1');
save_system(mdl);
end
