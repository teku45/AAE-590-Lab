% blimp_1
b1.ID = 5;
b1.mode = 0;
b1.DIRL = 0;
b1.DIRR =0;
b1.PWMV = 0;
b1.PWML = 0;
b1.PWMR = 0;
b1.update = 0;
b1.disable = 0;
b1.ref_height = 1000;
% forward angle [deg]
b1.fwd_deg = 20;
% distance marin [mm]
b1.dist_mar = 600;
b1.curWP = 1;
% PID gain
b1.spd = 0;
b1.espd = 0;
b1.vert_distance = 0;
b1.vert_distance_old = 0;
b1.vert_distance_cum = 0;
b1.dangle_cum = 0;
b1.dangle_old = 0;
b1.espd_old = 0;
b1.espd_cum = 0;
b1.evertical_old = 0;
b1.pos_old = [0 0 0];
b1.evertical_cum = 0;
b1.e_heading_old = 0;
b1.e_heading_cum = 0;

% heading
b1.pgain_h = 0;
b1.igain_h = 0;
b1.dgain_h = 0;
% velocity
b1.pgain_v = 0;
b1.igain_v = 0;
b1.dgain_v = 0;
% altitude
b1.pgain_vert = 0;
b1.igain_vert = 0;
b1.dgain_vert = 0;
b1.spinDir = 0;

b1.height_check = 0;
b1.forward_spd = 100;