function [wr] = wr_control_spd(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML = 0;
    PWMR = 0;

    Kp = 0.0305;
    Ki = 0.006722;
    Kd = 0.00047;

    %v_curr = pdist([wr.pos; wr.pos_old]) / time.dt
    v_curr = sqrt((wr.pos(2)-wr.pos_old(2))^2 + (wr.pos(1)-wr.pos_old(1))^2) / time.dt;
    wr.espd = wr.forward_spd - v_curr;
    %[wr.espd]
    espd_deriv = (wr.espd -  wr.espd_old) / time.dt;
    wr.espd_cum = wr.espd_cum + wr.espd*(1-wr.obs_mode);
    PWM = Kp * wr.espd + Ki * wr.espd_cum + Kd * espd_deriv;
    % Food for thought: is this the error to be added to v_curr or just the
    % new wheel velocity 

    % Update old error
    wr.espd_old = wr.espd;

    PWML = PWM*(1-wr.obs_mode);
    PWMR = PWM*(1-wr.obs_mode);

    if isnan(wr.espd_cum)
        wr.espd_cum=0;
    end

    % setting the PWM limits, keep the codes here
    %PWML = min(254, PWML);
    PWML = rescale(PWML,30,254,'InputMin',0,'InputMax',255);
    wr.PWML = uint8(max(0, PWML));
    %PWMR = min(254, PWMR);
    PWMR = rescale(PWMR,30,254,'InputMin',0,'InputMax',255);
    wr.PWMR = uint8(max(0, PWMR));
    PWML;
    wr.pos_old = wr.pos;


    scatter(time.curr,v_curr,'k'); hold on;
    scatter(time.curr,wr.forward_spd,'r');
    drawnow;
end