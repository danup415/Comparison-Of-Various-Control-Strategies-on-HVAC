function [CO2, PM25, doas_on] = iaq_model(T_out, RH, h24, PM25_out, hvac_mode)
persistent co2_st pm_st;
if isempty(co2_st); co2_st=450; pm_st=15; end
V=270.75; Q_inf=0.03760; Q_oa_base=0.0600; dt=3600; CO2_out=420;
occ_s=[0.9,0.9,0.9,0.9,0.9,0.8,0.6,0.3,0.2,0.1,0.1,0.2,0.5,0.5,0.3,0.2,0.3,0.5,0.7,0.8,0.9,0.9,0.9,0.9];
ck_s=[0,0,0,0,0,0,1,1,0,0,0,0,0,0,0,0,0,0,1,1,0,0,0,0];
h_idx = max(1,min(24,round(h24)));
doas_rh = double(RH>65 && T_out>22);
doas_hvac = double(hvac_mode>=3);
doas_on = double(doas_rh || doas_hvac);
Q_oa = Q_oa_base*(1 + 0.5*double(hvac_mode>=3));
G_co2 = 4*0.0052e-3*occ_s(h_idx);
Q_v = Q_inf + Q_oa*doas_on;
dco2 = (G_co2-Q_v*(co2_st/1e6-CO2_out/1e6))/V;
co2_st = max(CO2_out,min(3000,co2_st+dco2*dt*1e6));
recirc = 0.30*(1+0.3*double(hvac_mode>=2));
G_pm = 150/3600*ck_s(h_idx)+5/3600;
dpm = (Q_inf*PM25_out+G_pm*V-recirc*0.9995*pm_st-0.5/3600*pm_st*V)/V;
pm_st = max(0,min(300,pm_st+dpm*dt));
CO2=co2_st; PM25=pm_st;
end
