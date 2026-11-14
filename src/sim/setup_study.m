function ctx = setup_study(cfg)
%SETUP_STUDY  Load weather, build inputs, the "true" plant and the controllers' nominal model.
w = read_epw(cfg.epw_file);
ctx.w = w;
ctx.D = build_inputs(cfg, w);
nm = build_model(cfg);                       % nominal model used by every model-based controller
nm.gain_solar = 1; nm.gain_int = 1;
mm = cfg.mismatch;
if mm.enable
    pm = build_model(cfg, struct('Ca', mm.Ca, 'Cm', mm.Cm, 'UA', mm.UA));
    pm.gain_solar = mm.solar; pm.gain_int = mm.int;
else
    pm = nm;
end
ctx.nm = nm; ctx.pm = pm;
end
