function cfg = validation_config()
%VALIDATION_CONFIG  Configuration shared by the RC model and the EnergyPlus model for the
%   cross-check: base building, no overhang shading, no DOAS/IEC, plant = nominal model.
cfg = default_config();
cfg.bld.shade_month = zeros(1,12);
cfg.doas.mode = 'off';
cfg.mismatch.enable = false;
cfg.scenario = 'validation';
end
