function make_energyplus_case(outdir)
%MAKE_ENERGYPLUS_CASE  Write an EnergyPlus (v24.2) model equivalent to the study's building,
%   for an independent cross-check of the reduced-order RC model.
%
%   Two IDFs are written to outdir (default validation/energyplus/case):
%     pune_free.idf   free-floating (no HVAC)
%     pune_ideal.idf  HVACTemplate ideal loads, cooling only, same setpoint schedule
%   plus schedule CSVs with the SAME 15-min internal gains and setpoints used by the RC
%   model, so that only the building physics differs. Simplifications applied to BOTH
%   models for this comparison (validation_config): no overhang shading, no DOAS/IEC,
%   no plant/model mismatch.
%   Run with run_energyplus_comparison.m (needs EnergyPlus 24.2 installed).
if nargin < 1, outdir = fullfile(fileparts(mfilename('fullpath')), 'case'); end
if ~exist(outdir, 'dir'), mkdir(outdir); end
cfg = validation_config(); ctx = setup_study(cfg); D = ctx.D; b = cfg.bld;

% ---------- schedule files (15-min, 35040 rows) ----------
fid = fopen(fullfile(outdir, 'gains_W.csv'), 'w'); fprintf(fid, 'Z1,Z2,Z3\n');
fprintf(fid, '%.3f,%.3f,%.3f\n', D.Qint'); fclose(fid);
fid = fopen(fullfile(outdir, 'cool_sp.csv'), 'w'); fprintf(fid, 'Z1,Z2,Z3\n');
fprintf(fid, '%.2f,%.2f,%.2f\n', D.sp'); fclose(fid);
copyfile(cfg.epw_file, fullfile(outdir, 'weather.epw'));

for mode = {'free', 'ideal'}
    fid = fopen(fullfile(outdir, sprintf('pune_%s.idf', mode{1})), 'w');
    p = @(varargin) fprintf(fid, varargin{:});
    p('Version, 24.2;\n');
    p('SimulationControl, No, No, No, No, Yes, No, 1;\n');
    p('Building, Pune3BHK, 0, Suburbs, 0.04, 0.4, MinimalShadowing, 25, 6;\n');
    p('Timestep, 4;\n');
    p('RunPeriod, Year, 1, 1, , 12, 31, , Sunday, No, No, No, Yes, Yes, No;\n');
    p('GlobalGeometryRules, UpperLeftCorner, Counterclockwise, World;\n');
    p('ScheduleTypeLimits, AnyNumber;\n');
    p('Schedule:Constant, AlwaysOn, AnyNumber, 1;\n');
    p('Schedule:Constant, NoHeat, AnyNumber, -50;\n');
    % ---- materials (outside -> inside); U-values reproduce the RC model
    p('Material, PlasterOut, Smooth, 0.012, 0.72, 1860, 840, 0.9, %.2f, %.2f;\n', b.alpha_wall, b.alpha_wall);
    p('Material, PlasterIn, Smooth, 0.012, 0.72, 1860, 840, 0.9, 0.4, 0.4;\n');
    p('Material, AAC200, MediumRough, 0.200, 0.177, 550, 1000, 0.9, 0.6, 0.6;\n');
    p('Material, Screed40, Rough, 0.040, 1.40, 2000, 840, 0.9, %.2f, %.2f;\n', b.alpha_roof, b.alpha_roof);
    p('Material, XPS, Smooth, 0.0165, 0.034, 35, 1400, 0.9, 0.6, 0.6;\n');
    p('Material, RCC150, MediumRough, 0.150, 1.58, 2400, 880, 0.9, 0.6, 0.6;\n');
    p('Material, Tile50, Smooth, 0.050, 1.00, 2000, 840, 0.9, 0.6, 0.6;\n');
    p('Material, Brick115, Rough, 0.115, 0.19, 1920, 790, 0.9, 0.6, 0.6;\n');
    p('Material, Wood25, Smooth, 0.025, 0.15, 600, 1630, 0.9, 0.6, 0.6;\n');
    % frame fraction folded into SHGC: whole opening has U_glz, 75 % of it transmits solar
    p('WindowMaterial:SimpleGlazingSystem, Glz, %.2f, %.4f, 0.6;\n', b.U_glz, b.SHGC*(1-b.frame_frac));
    p('Construction, ExtWall, PlasterOut, AAC200, PlasterIn;\n');
    p('Construction, Roof, Screed40, XPS, RCC150, PlasterIn;\n');
    p('Construction, Floor, RCC150, Tile50;\n');
    p('Construction, Partition, PlasterIn, Brick115, PlasterIn;\n');
    p('Construction, Furniture, Wood25;\n');
    p('Construction, Window, Glz;\n');
    names = {'Z1','Z2','Z3'}; az = b.azimuth; H = b.height;
    A_part = [b.part_len(1); b.part_len(2)] * H;
    for z = 1:3
        A = b.area(z); V = A*H; a = sqrt(A); off = [0 60 120]*1;   % zones placed apart; MinimalShadowing
        p('Zone, %s, 0, %d, 0, 0, 1, 1, %.2f, %.2f;\n', names{z}, off(z), H, V);
        % roof and adiabatic floor (exact area)
        x0 = off(z);
        p('BuildingSurface:Detailed, %s_Roof, Roof, Roof, %s, , Outdoors, , SunExposed, WindExposed, 0, 4, %s;\n', ...
          names{z}, names{z}, vstr([x0 a H; x0 0 H; x0+a 0 H; x0+a a H]));
        p('BuildingSurface:Detailed, %s_Floor, Floor, Floor, %s, , Adiabatic, , NoSun, NoWind, 0, 4, %s;\n', ...
          names{z}, names{z}, vstr([x0 0 0; x0 a 0; x0+a a 0; x0+a 0 0]));
        % exterior walls by orientation, with centred windows (WWR)
        for f = 1:4
            L = b.ext_perim(z) * b.facade(z,f);
            if L <= 0, continue; end
            wn = sprintf('%s_Wall%d', names{z}, az(f));
            V4 = wallverts(x0 + a/2, a/2, az(f), L, H, 30);
            p('BuildingSurface:Detailed, %s, Wall, ExtWall, %s, , Outdoors, , SunExposed, WindExposed, 0.5, 4, %s;\n', ...
              wn, names{z}, vstr(V4));
            Ag = b.WWR*L*H; hw = 1.5; ww = Ag/hw;
            W4 = winverts(V4, L, H, ww, hw);
            p('FenestrationSurface:Detailed, %s_Win, Window, Window, %s, , 0.5, , 1, 4, %s;\n', wn, wn, vstr(W4));
        end
        p('InternalMass, %s_Furn, Furniture, %s, , %.3f;\n', names{z}, names{z}, b.kappa_furn*A/(600*1630*0.025));
        p('ZoneInfiltration:DesignFlowRate, %s_Inf, %s, AlwaysOn, AirChanges/Hour, , , , %.2f, 1, 0, 0, 0;\n', names{z}, names{z}, b.ACH_inf);
        p('Schedule:File, %s_GainSch, AnyNumber, gains_W.csv, %d, 1, 8760, Comma, No, 15, No;\n', names{z}, z);
        p('OtherEquipment, %s_Gains, None, %s, %s_GainSch, EquipmentLevel, 1, , , 0, %.2f, 0, 0, General;\n', ...
          names{z}, names{z}, names{z}, 1 - b.int_conv);
    end
    % partitions Z1|Z2 and Z2|Z3 (inter-zone, matching areas)
    pairs = [1 2; 2 3];
    for k = 1:2
        za = pairs(k,1); zb = pairs(k,2); L = b.part_len(k);
        Va = wallverts(off(za) + 5, 5, 90, L, H, 25);  Vb = flipud(Va);
        p('BuildingSurface:Detailed, P%d_%s, Wall, Partition, %s, , Surface, P%d_%s, NoSun, NoWind, 0, 4, %s;\n', ...
          k, names{za}, names{za}, k, names{zb}, vstr(Va));
        p('BuildingSurface:Detailed, P%d_%s, Wall, Partition, %s, , Surface, P%d_%s, NoSun, NoWind, 0, 4, %s;\n', ...
          k, names{zb}, names{zb}, k, names{za}, vstr(Vb));
    end
    if strcmp(mode{1}, 'ideal')
        for z = 1:3
            p('Schedule:File, %s_CoolSP, AnyNumber, cool_sp.csv, %d, 1, 8760, Comma, No, 15, No;\n', names{z}, z);
            p('HVACTemplate:Thermostat, %s_Tstat, NoHeat, , %s_CoolSP, ;\n', names{z}, names{z});
            p('HVACTemplate:Zone:IdealLoadsAirSystem, %s, %s_Tstat;\n', names{z}, names{z});
        end
        p('Output:Variable, *, Zone Ideal Loads Zone Sensible Cooling Rate, Timestep;\n');
    end
    p('Output:Variable, *, Zone Mean Air Temperature, Timestep;\n');
    p('Output:Variable, Environment, Site Outdoor Air Drybulb Temperature, Timestep;\n');
    p('OutputControl:Table:Style, Comma;\n');
    fclose(fid);
end
fprintf('EnergyPlus case written to %s\n', outdir);
end

function s = vstr(V)
c = cell(1, size(V,1));
for i = 1:size(V,1), c{i} = sprintf('%.4f, %.4f, %.4f', V(i,:)); end
s = strjoin(c, ', ');
end

function V = wallverts(cx, cy, az, L, H, dist)
% vertical rectangle facing azimuth az (deg from north), centred at dist along the normal
n = [sind(az) cosd(az) 0]; r = [-cosd(az) sind(az) 0];
P0 = [cx cy 0] + dist*n - (L/2)*r; z = [0 0 H];
V = [P0 + z; P0; P0 + L*r; P0 + L*r + z];            % UL, LL, LR, UR (CCW seen from outside)
end

function W = winverts(V, L, H, ww, hw)
r = (V(3,:) - V(2,:)) / L; z = [0 0 1];
P0 = V(2,:) + ((L - ww)/2)*r + ((H - hw)/2)*z;
W = [P0 + hw*z; P0; P0 + ww*r; P0 + ww*r + hw*z];
end
