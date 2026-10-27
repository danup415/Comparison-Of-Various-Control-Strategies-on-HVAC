function [Qz1,Qz2,Qz3,P_elec,COP_out,mode_out] = hvac_equip(Q_cool,T_out,RH,iec_cap,doas_P)
vrf_Qmax=11000.0; COP_r=4.20; T_ref=35; a=0.0350; b=0.000300;
T_min=15; T_max=48;
frac = [0.3158, 0.4737, 0.2105];
Q_iec=0; P_iec=0;
if T_out>28 && RH<50 && Q_cool>200
    Q_iec = iec_cap*0.5; P_iec = 150.0;
end
COP = max(1.5, COP_r*(1+a*(T_ref-T_out)+b*(T_ref-T_out)^2));
if T_out<T_min||T_out>T_max; COP=0; end
Q_tot = Q_cool + Q_iec;
P_vrf = 0;
if COP>0 && Q_cool>0; P_vrf = (Q_cool/1000)/COP; end
P_elec = P_vrf + P_iec/1000 + doas_P;
COP_out = COP;
Qz1 = -Q_tot*frac(1);
Qz2 = -Q_tot*frac(2);
Qz3 = -Q_tot*frac(3);
if Q_cool<50; mode_out=1;
elseif P_iec>0 && doas_P==0; mode_out=2;
elseif doas_P>0; mode_out=4;
else; mode_out=3; end
end
