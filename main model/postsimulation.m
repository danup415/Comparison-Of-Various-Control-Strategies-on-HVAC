%% ================================================================
%% Pune_HVAC_PostSim.m
%% POST-SIMULATION ANALYSIS — Run all 4 controllers, extract results,
%% plot everything, print full KPI table
%%
%% HOW TO USE:
%%   Step 1: Make sure Pune_HVAC_Complete.slx is open
%%   Step 2: Run this script — it simulates all 4 controllers
%%           automatically and collects results
%%
%% REQUIREMENTS: Pune_HVAC_Complete.slx must exist (run main script first)
%% ================================================================

clearvars; clc; close all;

mdl = 'Pune_HVAC_Complete';

%% ── Load model if not already open
if ~bdIsLoaded(mdl)
    if exist([mdl '.slx'],'file')
        load_system(mdl);
        fprintf('Loaded: %s.slx\n', mdl);
    else
        error('Pune_HVAC_Complete.slx not found. Run Pune_HVAC_Full_Simulink.m first.');
    end
end

fprintf('================================================================\n');
fprintf('  PUNE HVAC — POST-SIMULATION ANALYSIS\n');
fprintf('  Running all 4 controllers and comparing results\n');
fprintf('================================================================\n\n');

%% ── Constants
ctrl_names = {'Baseline','PID','MPC','RL-DQN'};
ctrl_ids   = {'baseline','pid','mpc','rl'};
ctrl_vals  = {'1','2','3','4'};   % CtrlSelect constant values
N          = 8760;
T_SET      = 24;
GRID_FAC   = 0.82;

colors = [0.43 0.44 0.45;   % Baseline — grey
          0.20 0.53 0.96;   % PID      — blue
          0.62 0.32 0.95;   % MPC      — purple
          0.15 0.75 0.30];  % RL-DQN   — green

R = struct();   % store all results here

%% ================================================================
%% SECTION 1 — RUN ALL 4 CONTROLLERS
%% ================================================================
fprintf('SECTION 1: Simulating all 4 controllers\n');
fprintf('%-10s %-8s %-10s %-10s %-10s %-10s\n',...
    'Controller','Time(s)','E(kWh)','Comfort%','RMSE°C','CO2peak');
fprintf('%s\n', repmat('─',1,62));

for ci = 1:4
    cid  = ctrl_ids{ci};
    cval = ctrl_vals{ci};

    % Switch controller
    set_param([mdl '/CtrlSelect'], 'Value', cval);

    % Run simulation
    tic_c = tic;
    simOut = sim(mdl);
    elapsed = toc(tic_c);

    % Extract timeseries data
    Tz1  = simOut.T_z1_out.Data;
    Tz2  = simOut.T_z2_out.Data;
    Tz3  = simOut.T_z3_out.Data;
    CO2  = simOut.CO2_out.Data;
    PM25 = simOut.PM25_out.Data;
    Pelec= simOut.P_elec_out.Data;   % kW
    COP  = simOut.COP_out.Data;
    mode = simOut.mode_out.Data;
    tout = simOut.tout;              % seconds

    % Trim/pad to exactly N hours if needed
    n_out = min(length(Tz2), N);
    Tz2  = Tz2(1:n_out);
    Tz1  = Tz1(1:n_out);
    Tz3  = Tz3(1:n_out);
    CO2  = CO2(1:n_out);
    PM25 = PM25(1:n_out);
    Pelec= Pelec(1:n_out);
    COP  = COP(1:n_out);
    mode = mode(1:n_out);

    % ── KPIs
    E_tot   = sum(Pelec);           % kWh (since Pelec is kW × 1h steps)
    h_comf  = sum(Tz2>=22 & Tz2<=26);
    h_cool  = sum(Tz2>26);
    h_heat  = sum(Tz2<22);
    rmse    = rms(Tz2 - T_SET);
    carbon  = E_tot * GRID_FAC;
    cost    = E_tot * 7.15;
    co2_pk  = max(CO2);
    pm_mean = mean(PM25);

    % Monthly energy
    mdays = [31,28,31,30,31,30,31,31,30,31,30,31];
    E_monthly = zeros(1,12);
    h_ptr = 1;
    for m = 1:12
        h_end = min(h_ptr + mdays(m)*24 - 1, n_out);
        E_monthly(m) = sum(Pelec(h_ptr:h_end));
        h_ptr = h_end + 1;
    end

    % Mode hours
    mode_h = arrayfun(@(m) sum(mode==m), 1:4);

    % Store
    R.(cid).Tz1      = Tz1;
    R.(cid).Tz2      = Tz2;
    R.(cid).Tz3      = Tz3;
    R.(cid).CO2      = CO2;
    R.(cid).PM25     = PM25;
    R.(cid).Pelec    = Pelec;
    R.(cid).COP      = COP;
    R.(cid).mode     = mode;
    R.(cid).tout     = tout;
    R.(cid).E_total  = E_tot;
    R.(cid).h_comf   = h_comf;
    R.(cid).h_cool   = h_cool;
    R.(cid).h_heat   = h_heat;
    R.(cid).rmse     = rmse;
    R.(cid).carbon   = carbon;
    R.(cid).cost     = cost;
    R.(cid).co2_peak = co2_pk;
    R.(cid).pm_mean  = pm_mean;
    R.(cid).E_monthly= E_monthly;
    R.(cid).mode_h   = mode_h;
    R.(cid).specific = E_tot / 95;

    fprintf('%-10s %7.1fs  %8.0f   %8.1f   %7.2f   %8.0f\n',...
        ctrl_names{ci}, elapsed, E_tot, h_comf/n_out*100, rmse, co2_pk);
end

save('Simulink_Results.mat','R','ctrl_names','ctrl_ids');
fprintf('\nSaved: Simulink_Results.mat\n\n');

%% ================================================================
%% SECTION 2 — PRINT FULL KPI TABLE
%% ================================================================
fprintf('================================================================\n');
fprintf('  FULL KPI TABLE — SIMULINK SIMULATION RESULTS\n');
fprintf('================================================================\n');
fprintf('%-38s %9s %9s %9s %9s\n','KPI','Baseline','PID','MPC','RL-DQN');
fprintf('%s\n', repmat('─',1,78));

bl = R.baseline;
rows = {
    'Annual energy (kWh/yr)',        'E_total',  '%.0f';
    'Specific energy (kWh/m²/yr)',   'specific', '%.1f';
    'Comfort 22–26°C (%)',           '',         '';   % computed specially
    'Cooling hours >26°C (h/yr)',    'h_cool',   '%d';
    'Heating hours <22°C (h/yr)',    'h_heat',   '%d';
    'RMSE from setpoint (°C)',       'rmse',     '%.2f';
    'Carbon emissions (kgCO₂/yr)',   'carbon',   '%.0f';
    'Electricity cost (₹/yr)',       'cost',     '%.0f';
    'CO₂ peak (ppm)',                'co2_peak', '%.0f';
    'PM₂.₅ mean indoor (µg/m³)',     'pm_mean',  '%.2f';
};

for r = 1:size(rows,1)
    label = rows{r,1}; field = rows{r,2}; fmt = rows{r,3};
    vals  = zeros(1,4);
    for ci=1:4
        if strcmp(field,'')
            vals(ci) = R.(ctrl_ids{ci}).h_comf / N * 100;
            fmt2 = '%.1f%%';
        else
            vals(ci) = R.(ctrl_ids{ci}).(field);
            fmt2 = [fmt];
        end
    end
    % Format with colour markers
    strs = arrayfun(@(v) sprintf(fmt2,v), vals,'UniformOutput',false);
    fprintf('%-38s %9s %9s %9s %9s\n', label, strs{:});
end

fprintf('%s\n', repmat('═',1,78));
fprintf('\nVs baseline savings:\n');
for ci = 2:4
    pct_E = (bl.E_total   - R.(ctrl_ids{ci}).E_total)   / bl.E_total * 100;
    pct_C = (R.(ctrl_ids{ci}).h_comf - bl.h_comf) / bl.h_comf * 100;
    fprintf('  %-8s → Energy: %+.1f%%   Comfort: %+.1f%%   RMSE: %.2f°C\n',...
        ctrl_names{ci}, pct_E, pct_C, R.(ctrl_ids{ci}).rmse);
end

%% ================================================================
%% SECTION 3 — FIGURE 1: Annual Zone 2 Temperature (all controllers)
%% ================================================================
fprintf('\nGenerating figures...\n');

h_vec = (1:N)';
MONTHS_LBL = {'Jan','Feb','Mar','Apr','May','Jun','Jul','Aug','Sep','Oct','Nov','Dec'};
month_ticks = [1, 745, 1417, 2161, 2881, 3625, 4369, 5137, 5881, 6625, 7345, 8017];

figure('Name','Fig A — Zone 2 Annual Temperature','Position',[50 50 1400 600]);

% Season shading
season_colors = [0.22 0.48 0.73;  % winter blue
                 0.83 0.46 0.09;  % pre-monsoon orange
                 0.06 0.44 0.33;  % monsoon green
                 0.33 0.29 0.72]; % post-monsoon purple
season_bounds = [1 720; 721 3624; 3625 6552; 6553 8016];
season_labels = {'Winter','Pre-monsoon','Monsoon','Post-monsoon'};

subplot(3,1,1);
hold on;
for s = 1:4
    patch([season_bounds(s,1) season_bounds(s,2) season_bounds(s,2) season_bounds(s,1)],...
        [8 8 50 50], season_colors(s,:),'FaceAlpha',0.10,'EdgeColor','none');
end
for ci = 1:4
    lw = [0.7 1.2 1.2 2.0];
    ls = {'--','-','-','-'};
    plot(h_vec, R.(ctrl_ids{ci}).Tz2,'Color',colors(ci,:),...
        'LineWidth',lw(ci),'LineStyle',ls{ci});
end
yline(26,'--r','LineWidth',1,'Label','26°C cool','LabelHorizontalAlignment','left');
yline(22,'--b','LineWidth',1,'Label','22°C heat','LabelHorizontalAlignment','left');
yline(T_SET,':k','LineWidth',1.2);
xlim([1 N]); ylim([8 52]);
xticks(month_ticks); xticklabels(MONTHS_LBL);
ylabel('Temperature (°C)'); title('Zone 2 (Living+Dining) — All 4 controllers');
legend([ctrl_names season_labels],'Location','northeast','FontSize',7,...
    'NumColumns',2);
grid on; box on;

subplot(3,1,2);
hold on;
for ci = 1:4
    plot(h_vec, R.(ctrl_ids{ci}).Tz1,'Color',colors(ci,:),'LineWidth',0.7);
end
yline(26,'--r'); yline(22,'--b');
xlim([1 N]); ylim([8 52]);
xticks(month_ticks); xticklabels(MONTHS_LBL);
ylabel('Temperature (°C)'); title('Zone 1 — Master Bedroom');
legend(ctrl_names,'Location','northeast','FontSize',8,'NumColumns',2);
grid on;

subplot(3,1,3);
hold on;
for ci = 1:4
    plot(h_vec, R.(ctrl_ids{ci}).Tz3,'Color',colors(ci,:),'LineWidth',0.7);
end
yline(26,'--r'); yline(22,'--b');
xlim([1 N]); ylim([8 52]);
xticks(month_ticks); xticklabels(MONTHS_LBL);
ylabel('Temperature (°C)'); title('Zone 3 — 2nd Bedroom');
legend(ctrl_names,'Location','northeast','FontSize',8,'NumColumns',2);
grid on;

sgtitle('Annual Zone Temperature Profiles — Simulink Simulation Results',...
    'FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_A_ZoneTemperatures.png');
fprintf('  Saved: SimFig_A_ZoneTemperatures.png\n');

%% ================================================================
%% SECTION 4 — FIGURE B: Tracking Error + RMSE
%% ================================================================
figure('Name','Fig B — Tracking Error','Position',[50 50 1400 500]);

for ci = 1:4
    subplot(1,4,ci);
    err = R.(ctrl_ids{ci}).Tz2 - T_SET;
    % Duration above/below
    n_above = sum(err >  2);
    n_below = sum(err < -2);

    plot(h_vec, err,'Color',colors(ci,:),'LineWidth',0.6);
    hold on;
    yline(0,  '--k','LineWidth',1);
    yline( 2, '--r','LineWidth',0.8,'Label','+2°C');
    yline(-2, '--b','LineWidth',0.8,'Label','-2°C');
    fill([1 N N 1],[ 2  2 20 20],[1 0.8 0.8],'FaceAlpha',0.15,'EdgeColor','none');
    fill([1 N N 1],[-20 -20 -2 -2],[0.8 0.8 1],'FaceAlpha',0.15,'EdgeColor','none');

    xlabel('Hour of year'); ylabel('Error (°C)');
    title(sprintf('\\bf%s\\rm\nRMSE = %.2f°C', ctrl_names{ci},...
        R.(ctrl_ids{ci}).rmse),'FontSize',10);
    xlim([1 N]);
    text(200, 14, sprintf('+2σ band: %dh', n_above),'FontSize',7,'Color',[0.7 0 0]);
    text(200,-14, sprintf('-2σ band: %dh', n_below),'FontSize',7,'Color',[0 0 0.7]);
    grid on;
    set(gca,'Color',[0.98 0.98 0.98]);
end
sgtitle('Zone 2 Tracking Error from 24°C Setpoint','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_B_TrackingError.png');
fprintf('  Saved: SimFig_B_TrackingError.png\n');

%% ================================================================
%% SECTION 5 — FIGURE C: Energy Analysis
%% ================================================================
figure('Name','Fig C — Energy Analysis','Position',[50 50 1400 600]);

% Monthly energy bars
subplot(2,2,1);
E_mat = zeros(12,4);
for ci=1:4; E_mat(:,ci)=R.(ctrl_ids{ci}).E_monthly'; end
b = bar(E_mat,'grouped');
for ci=1:4; b(ci).FaceColor=colors(ci,:); b(ci).FaceAlpha=0.85; end
xlabel('Month'); ylabel('Energy (kWh)');
title('Monthly energy consumption');
xticklabels(MONTHS_LBL); legend(ctrl_names,'FontSize',8,'Location','northwest');
grid on;

% Annual energy bar
subplot(2,2,2);
E_ann = arrayfun(@(ci) R.(ctrl_ids{ci}).E_total, 1:4);
b2 = bar(E_ann,'FaceColor','flat');
for ci=1:4; b2.CData(ci,:)=colors(ci,:); end
xticklabels(ctrl_names); ylabel('Annual energy (kWh/yr)');
title('Annual energy comparison');
grid on;
for ci=1:4
    pct = (E_ann(ci)-E_ann(1))/E_ann(1)*100;
    col = [0 0.5 0]; if pct>0; col=[0.8 0 0]; end
    text(ci, E_ann(ci)+100, sprintf('%.0f\n(%+.0f%%)',E_ann(ci),pct),...
        'HorizontalAlignment','center','FontSize',8,'Color',col);
end

% COP profiles (hourly mean)
subplot(2,2,3);
hrs = 0:23;
hold on;
for ci=1:4
    cop_h = arrayfun(@(h) mean(R.(ctrl_ids{ci}).COP(mod(0:N-1,24)==h)),...
        0:23);
    plot(hrs, cop_h,'Color',colors(ci,:),'LineWidth',1.5,...
        'Marker','o','MarkerSize',3);
end
tod_tariff = [5.20 5.20 5.20 5.20 5.20 5.20 8.95 8.95 8.95 ...
              7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 7.15 ...
              8.95 8.95 8.95 8.95 8.95 5.20];
yyaxis right;
bar(hrs, tod_tariff,'FaceColor',[1 0.85 0.6],'FaceAlpha',0.5,'EdgeColor','none');
ylabel('ToD tariff (₹/kWh)'); ylim([4 11]);
yyaxis left;
ylabel('Mean COP'); xlabel('Hour of day');
title('Hourly mean VRF COP + ToD tariff');
legend([ctrl_names {'Tariff'}],'FontSize',7,'Location','northwest');
xlim([-0.5 23.5]); grid on;

% Load duration curves
subplot(2,2,4);
hold on;
for ci=1:4
    P_sorted = sort(R.(ctrl_ids{ci}).Pelec,'descend');
    plot(1:N, P_sorted,'Color',colors(ci,:),'LineWidth',1.5);
end
xlabel('Hours (sorted)'); ylabel('Electrical power (kW)');
title('VRF load duration curve');
legend(ctrl_names,'FontSize',8,'Location','northeast');
grid on;

sgtitle('Energy Analysis — All Controllers','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_C_EnergyAnalysis.png');
fprintf('  Saved: SimFig_C_EnergyAnalysis.png\n');

%% ================================================================
%% SECTION 6 — FIGURE D: Comfort + Mode Distribution
%% ================================================================
figure('Name','Fig D — Comfort','Position',[50 50 1400 600]);

% Stacked comfort bar
subplot(2,2,1);
cdata = zeros(3,4);
for ci=1:4
    n_out = length(R.(ctrl_ids{ci}).Tz2);
    cdata(:,ci) = [R.(ctrl_ids{ci}).h_cool/n_out*100;
                   R.(ctrl_ids{ci}).h_comf/n_out*100;
                   R.(ctrl_ids{ci}).h_heat/n_out*100];
end
b3 = bar(cdata','stacked');
b3(1).FaceColor=[0.95 0.32 0.22]; b3(2).FaceColor=[0.22 0.75 0.35];
b3(3).FaceColor=[0.22 0.40 0.90];
xticklabels(ctrl_names); ylabel('% of annual hours');
title('Comfort distribution');
legend({'Cooling needed >26°C','Comfortable 22–26°C','Heating needed <22°C'},...
    'FontSize',8,'Location','southoutside');
ylim([0 102]); grid on;
for ci=1:4
    text(ci, 101, sprintf('\\bf%.0f%%',cdata(2,ci)),...
        'HorizontalAlignment','center','FontSize',10,'Color',colors(ci,:));
end

% Temperature duration curve
subplot(2,2,2);
hold on;
for ci=1:4
    T_sorted = sort(R.(ctrl_ids{ci}).Tz2,'descend');
    plot(1:N, T_sorted,'Color',colors(ci,:),'LineWidth',1.5);
end
yline(26,'--r','26°C','LineWidth',1.2);
yline(22,'--b','22°C','LineWidth',1.2);
xlabel('Hours (sorted)'); ylabel('Zone 2 temperature (°C)');
title('Temperature duration curve');
legend(ctrl_names,'FontSize',8); grid on;

% HVAC mode distribution (pie for each)
mode_names_lbl = {'Free cool','IEC+VRF','VRF only','VRF+DOAS'};
for ci = 1:4
    subplot(2,4,4+ci);
    mh = R.(ctrl_ids{ci}).mode_h;
    % Only show modes with >1% share
    valid = (mh/sum(mh)*100) > 1;
    if sum(valid) > 0
        pie(mh(valid), mode_names_lbl(valid));
    else
        pie([1 0 0 0]); % fallback
    end
    title(sprintf('\\bf%s\\rm\nModes',ctrl_names{ci}),'FontSize',9);
end

sgtitle('Comfort & HVAC Mode Analysis','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_D_Comfort.png');
fprintf('  Saved: SimFig_D_Comfort.png\n');

%% ================================================================
%% SECTION 7 — FIGURE E: IAQ (CO2 + PM2.5)
%% ================================================================
figure('Name','Fig E — IAQ','Position',[50 50 1400 550]);

subplot(2,2,1);
hold on;
for ci=1:4
    plot(h_vec, R.(ctrl_ids{ci}).CO2,'Color',colors(ci,:),'LineWidth',0.7);
end
yline(1420,'--k','ASHRAE 1420 ppm','LineWidth',1.2,...
    'LabelHorizontalAlignment','left');
yline(420,'--g','Outdoor 420 ppm');
xlabel('Hour of year'); ylabel('CO₂ (ppm)');
title('Annual indoor CO₂ profile');
legend(ctrl_names,'FontSize',8,'Location','northwest');
xticks(month_ticks); xticklabels(MONTHS_LBL);
xlim([1 N]); grid on;

subplot(2,2,2);
hold on;
for ci=1:4
    plot(h_vec, R.(ctrl_ids{ci}).PM25,'Color',colors(ci,:),'LineWidth',0.7);
end
yline(15,'--r','WHO 15 µg/m³','LineWidth',1.2);
yline(60,'--k','NAAQS 60 µg/m³');
xlabel('Hour of year'); ylabel('PM₂.₅ (µg/m³)');
title('Annual indoor PM₂.₅ (HEPA H13 active)');
legend(ctrl_names,'FontSize',8,'Location','northeast');
xticks(month_ticks); xticklabels(MONTHS_LBL);
xlim([1 N]); grid on;

% CO2 exceedance summary
subplot(2,2,3);
co2_exc = arrayfun(@(ci) sum(R.(ctrl_ids{ci}).CO2 > 1420), 1:4);
b5 = bar(co2_exc,'FaceColor','flat');
for ci=1:4; b5.CData(ci,:)=colors(ci,:); end
xticklabels(ctrl_names); ylabel('Hours above ASHRAE limit');
title('CO₂ exceedance hours (>1420 ppm)');
grid on;
for ci=1:4
    text(ci, co2_exc(ci)+2, sprintf('%dh',co2_exc(ci)),...
        'HorizontalAlignment','center','FontSize',9);
end

% PM2.5 mean
subplot(2,2,4);
pm_means = arrayfun(@(ci) R.(ctrl_ids{ci}).pm_mean, 1:4);
b6 = bar(pm_means,'FaceColor','flat');
for ci=1:4; b6.CData(ci,:)=colors(ci,:); end
xline(2.5,'--r'); % WHO line goes on same bar scale
yline(15,'--r','WHO 15');
xticklabels(ctrl_names); ylabel('Mean PM₂.₅ (µg/m³)');
title('Mean annual indoor PM₂.₅');
grid on;
for ci=1:4
    text(ci, pm_means(ci)+0.1, sprintf('%.1f',pm_means(ci)),...
        'HorizontalAlignment','center','FontSize',9);
end

sgtitle('Indoor Air Quality (IAQ) Analysis','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_E_IAQ.png');
fprintf('  Saved: SimFig_E_IAQ.png\n');

%% ================================================================
%% SECTION 8 — FIGURE F: Weekly detail view (pre-monsoon + monsoon)
%% ================================================================
figure('Name','Fig F — Weekly Detail','Position',[50 50 1400 600]);

% Pre-monsoon week (hour 2880 ≈ Apr 21)
wk1 = 2880:2880+167;
% Monsoon week (hour 4380 ≈ Jul 1)
wk2 = 4380:4380+167;
hrs_wk = 0:167;

for row = 1:2
    wk = wk1; wk_name = 'Pre-monsoon week (Apr 21–28)';
    if row==2; wk = wk2; wk_name = 'Monsoon week (Jul 1–8)'; end
    wk = wk(wk <= N);

    subplot(2,3,(row-1)*3+1);
    hold on;
    for ci=1:4
        plot(hrs_wk(1:length(wk)), R.(ctrl_ids{ci}).Tz2(wk),...
            'Color',colors(ci,:),'LineWidth',1.5+(ci==4)*0.5);
    end
    yline(26,'--r','LineWidth',1); yline(22,'--b','LineWidth',1);
    yline(T_SET,':k','LineWidth',1.2);
    xlabel('Hour'); ylabel('Zone 2 Temp (°C)');
    title(['Temp — ' wk_name]); legend(ctrl_names,'FontSize',7);
    grid on; xlim([0 167]);

    subplot(2,3,(row-1)*3+2);
    hold on;
    for ci=1:4
        plot(hrs_wk(1:length(wk)), R.(ctrl_ids{ci}).Pelec(wk),...
            'Color',colors(ci,:),'LineWidth',1.2);
    end
    xlabel('Hour'); ylabel('Electrical power (kW)');
    title(['VRF power — ' wk_name]);
    legend(ctrl_names,'FontSize',7); grid on; xlim([0 167]);

    subplot(2,3,(row-1)*3+3);
    hold on;
    for ci=1:4
        plot(hrs_wk(1:length(wk)), R.(ctrl_ids{ci}).CO2(wk),...
            'Color',colors(ci,:),'LineWidth',1.2);
    end
    yline(1420,'--k','ASHRAE limit');
    xlabel('Hour'); ylabel('CO₂ (ppm)');
    title(['CO₂ — ' wk_name]);
    legend(ctrl_names,'FontSize',7); grid on; xlim([0 167]);
end

sgtitle('Weekly Detail View — Pre-monsoon & Monsoon','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_F_WeeklyDetail.png');
fprintf('  Saved: SimFig_F_WeeklyDetail.png\n');

%% ================================================================
%% SECTION 9 — FIGURE G: Final KPI Radar + Summary
%% ================================================================
figure('Name','Fig G — KPI Summary','Position',[50 50 1400 650]);

% Radar chart (normalised vs baseline)
subplot(2,3,1);
kpi_fields  = {'E_total','h_comf','h_heat','carbon','rmse','pm_mean'};
kpi_labels2 = {'Energy','Comfort','Heating-h','Carbon','RMSE','PM2.5'};
% For comfort: higher is better → invert for radar
bl_vals = cellfun(@(f) R.baseline.(f), kpi_fields);

theta = linspace(0, 2*pi, length(kpi_fields)+1);
theta(end) = [];
hold on;
for ci=1:4
    vals = cellfun(@(f) R.(ctrl_ids{ci}).(f), kpi_fields);
    % Normalise: ratio to baseline (comfort inverted)
    ratios = vals ./ bl_vals;
    ratios(2) = bl_vals(2) / vals(2);   % invert comfort (higher=better)
    ratios(end+1) = ratios(1);           % close polygon
    th = [theta, theta(1)];
    polarplot(th, ratios,'Color',colors(ci,:),'LineWidth',1.5+double(ci==4));
    hold on;
end
polarplot(theta([1:end 1]), ones(1,length(kpi_fields)+1),'--k','LineWidth',1);
set(gca,'ThetaTick',rad2deg(theta),'ThetaTickLabel',kpi_labels2);
title('Normalised KPIs (1.0=baseline)','FontSize',9);
legend(ctrl_names,'Location','southoutside','FontSize',7,'NumColumns',2);

% Carbon + cost bars
subplot(2,3,2);
carb_vals = arrayfun(@(ci) R.(ctrl_ids{ci}).carbon, 1:4);
cost_vals  = arrayfun(@(ci) R.(ctrl_ids{ci}).cost/1000, 1:4);
yyaxis left;
b7 = bar(1:4, carb_vals,'FaceColor','flat','BarWidth',0.4);
for ci=1:4; b7.CData(ci,:)=colors(ci,:); end
ylabel('Carbon (kgCO₂/yr)');
yyaxis right;
b8 = bar(1:4, cost_vals,'FaceColor','flat','BarWidth',0.3);
for ci=1:4; b8.CData(ci,:)=colors(ci,:); end
b8.FaceAlpha=0.5;
ylabel('Cost (₹k/yr)');
xticklabels(ctrl_names); title('Carbon & Cost'); grid on;

% RMSE comparison
subplot(2,3,3);
rmse_v = arrayfun(@(ci) R.(ctrl_ids{ci}).rmse, 1:4);
bR = bar(rmse_v,'FaceColor','flat');
for ci=1:4; bR.CData(ci,:)=colors(ci,:); end
xticklabels(ctrl_names); ylabel('RMSE (°C)');
title('Setpoint tracking RMSE — Zone 2');
grid on;
for ci=1:4
    text(ci, rmse_v(ci)+0.05, sprintf('%.2f°C',rmse_v(ci)),...
        'HorizontalAlignment','center','FontSize',9,'FontWeight','bold');
end

% Scatter: RMSE vs Energy
subplot(2,3,4);
hold on;
for ci=1:4
    scatter(R.(ctrl_ids{ci}).E_total, R.(ctrl_ids{ci}).rmse, 120,...
        colors(ci,:),'filled');
    text(R.(ctrl_ids{ci}).E_total+30, R.(ctrl_ids{ci}).rmse+0.02,...
        ctrl_names{ci},'FontSize',9,'Color',colors(ci,:));
end
xlabel('Annual energy (kWh/yr)'); ylabel('RMSE (°C)');
title('Energy vs comfort trade-off');
% Ideal region: low energy + low RMSE
patch([0 max(arrayfun(@(ci)R.(ctrl_ids{ci}).E_total,1:4))*0.8 ...
       max(arrayfun(@(ci)R.(ctrl_ids{ci}).E_total,1:4))*0.8 0],...
    [0 0 2 2],[0.2 0.8 0.3],'FaceAlpha',0.08,'EdgeColor','none');
text(100, 0.3,'Ideal\nzone','FontSize',8,'Color',[0 0.5 0]);
grid on;

% Comfort % vertical
subplot(2,3,5);
comf_v = arrayfun(@(ci) R.(ctrl_ids{ci}).h_comf/N*100, 1:4);
bC = bar(comf_v,'FaceColor','flat');
for ci=1:4; bC.CData(ci,:)=colors(ci,:); end
xticklabels(ctrl_names); ylabel('Comfort hours (%)');
title('Time in comfort band 22–26°C');
ylim([0 100]); grid on;
for ci=1:4
    text(ci, comf_v(ci)+1, sprintf('%.1f%%',comf_v(ci)),...
        'HorizontalAlignment','center','FontSize',10,'FontWeight','bold',...
        'Color',colors(ci,:));
end

% Text summary
subplot(2,3,6);
axis off;
y = 0.97;
text(0.5, y,'SIMULATION RESULTS SUMMARY','HorizontalAlignment','center',...
    'FontWeight','bold','FontSize',11); y=y-0.07;
text(0.5, y,sprintf('Model: %s.slx',mdl),'HorizontalAlignment','center',...
    'FontSize',8,'Color',[0.4 0.4 0.4]); y=y-0.08;
for ci=1:4
    cid2=ctrl_ids{ci};
    line1=sprintf('\\bf%s\\rm',ctrl_names{ci});
    line2=sprintf('  E=%.0f kWh/yr (%.1f kWh/m²)',R.(cid2).E_total,R.(cid2).specific);
    line3=sprintf('  Comfort=%.0f%%  RMSE=%.2f°C',R.(cid2).h_comf/N*100,R.(cid2).rmse);
    line4=sprintf('  CO₂ peak=%.0f ppm  Carbon=%.0f kgCO₂',R.(cid2).co2_peak,R.(cid2).carbon);
    text(0.05,y,line1,'FontSize',9,'Color',colors(ci,:),'FontWeight','bold');  y=y-0.055;
    text(0.05,y,line2,'FontSize',8,'Color',[0.2 0.2 0.2]);  y=y-0.05;
    text(0.05,y,line3,'FontSize',8,'Color',[0.2 0.2 0.2]);  y=y-0.05;
    text(0.05,y,line4,'FontSize',8,'Color',[0.2 0.2 0.2]);  y=y-0.06;
end
text(0.05,y,sprintf('Grid: %.2f kgCO₂/kWh | Tariff: ₹7.15/kWh avg',GRID_FAC),...
    'FontSize',7,'Color',[0.5 0.5 0.5]);

sgtitle('Final KPI Summary — All Controllers','FontWeight','bold','FontSize',12);
saveas(gcf,'SimFig_G_KPISummary.png');
fprintf('  Saved: SimFig_G_KPISummary.png\n');

%% ================================================================
%% SECTION 10 — INTERACTIVE SCOPE SETUP GUIDE (printed)
%% ================================================================
fprintf('\n================================================================\n');
fprintf('  HOW TO VIEW RESULTS IN SIMULINK\n');
fprintf('================================================================\n\n');
fprintf('METHOD 1 — Scope blocks (inside the model):\n');
fprintf('  1. Open Pune_HVAC_Complete.slx\n');
fprintf('  2. Double-click "Scope_Temps" → shows Tz1, Tz2, Tz3\n');
fprintf('  3. Double-click "Scope_IAQ"   → shows CO2, PM2.5\n\n');

fprintf('METHOD 2 — Plot from workspace after sim():\n');
fprintf('  >> out = sim(''Pune_HVAC_Complete'');\n');
fprintf('  >> t = out.tout / 3600;           %% hours\n');
fprintf('  >> Tz2 = out.T_z2_out.Data;\n');
fprintf('  >> figure; plot(t, Tz2); xlabel(''Hour''); ylabel(''Temp (C)'');\n');
fprintf('  >> yline(26,''--r''); yline(22,''--b''); grid on;\n\n');

fprintf('METHOD 3 — Switch controller and compare:\n');
fprintf('  >> set_param(''Pune_HVAC_Complete/CtrlSelect'',''Value'',''1'');  %% Baseline\n');
fprintf('  >> out1 = sim(''Pune_HVAC_Complete'');\n');
fprintf('  >> set_param(''Pune_HVAC_Complete/CtrlSelect'',''Value'',''4'');  %% RL-DQN\n');
fprintf('  >> out4 = sim(''Pune_HVAC_Complete'');\n');
fprintf('  >> t = out1.tout/3600;\n');
fprintf('  >> figure; hold on;\n');
fprintf('  >> plot(t, out1.T_z2_out.Data, ''--k'', ''DisplayName'',''Baseline'');\n');
fprintf('  >> plot(t, out4.T_z2_out.Data, ''g'',  ''DisplayName'',''RL-DQN'');\n');
fprintf('  >> legend; grid on;\n\n');

fprintf('METHOD 4 — Run this script (already done):\n');
fprintf('  >> Pune_HVAC_PostSim    %% auto-runs all 4 → all figures\n\n');

fprintf('CONTROLLER SELECT VALUES:\n');
fprintf('  1 = Baseline (rule-based)    3 = MPC (predictive QP)\n');
fprintf('  2 = PID (3-zone + windup)    4 = RL-DQN (trained DQN)\n\n');

fprintf('OUTPUT SIGNALS (in workspace after sim):\n');
fprintf('  out.T_z1_out.Data   → Zone 1 temp (°C)\n');
fprintf('  out.T_z2_out.Data   → Zone 2 temp (°C)  [primary KPI zone]\n');
fprintf('  out.T_z3_out.Data   → Zone 3 temp (°C)\n');
fprintf('  out.CO2_out.Data    → Indoor CO₂ (ppm)\n');
fprintf('  out.PM25_out.Data   → Indoor PM₂.₅ (µg/m³)\n');
fprintf('  out.P_elec_out.Data → Total electrical power (kW)\n');
fprintf('  out.COP_out.Data    → VRF COP (dimensionless)\n');
fprintf('  out.mode_out.Data   → HVAC mode (1=free,2=IEC,3=VRF,4=DOAS)\n');
fprintf('  out.tout            → Time vector (seconds)\n\n');

fprintf('ALL FIGURES SAVED:\n');
imgs = {'SimFig_A_ZoneTemperatures','SimFig_B_TrackingError',...
        'SimFig_C_EnergyAnalysis','SimFig_D_Comfort',...
        'SimFig_E_IAQ','SimFig_F_WeeklyDetail','SimFig_G_KPISummary'};
for k=1:length(imgs)
    fprintf('  %s.png\n',imgs{k});
end

fprintf('\nSaved: Simulink_Results.mat\n');
fprintf('\n████████████  ANALYSIS COMPLETE  ████████████\n\n');