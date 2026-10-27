function Q_out = ctrl_mux(Q_bl,Q_pid,Q_mpc,Q_rl,sel)
switch round(sel)
    case 1; Q_out = Q_bl;
    case 2; Q_out = Q_pid;
    case 3; Q_out = Q_mpc;
    case 4; Q_out = Q_rl;
    otherwise; Q_out = 0;
end
end
