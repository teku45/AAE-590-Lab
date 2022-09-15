function [wr] = wr_control_spd(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML = 0;
    PWMR = 0;

    Kp = 0.805;
    Ki = 3.722;
    Kd = 0.037;

    % setting the PWM limits, keep the codes here
    v_curr = pdist([wr.pos; wr.pos_old]) / time.dt;
    wr.espd = wr.forward_spd - v_curr;

    espd_deriv = (wr.espd -  wr.espd_old) / time.dt;
    wr.espd_cum = wr.espd_cum + wr.espd;
    PWM = Kp * wr.espd + Ki * wr.espd_cum + Kd * espd_deriv;
    % Food for thought: is this the error to be added to v_curr or just the
    % new wheel velocity 

    % Update old error
    wr.espd_old = wr.espd;

    PWML = PWM;
    PWMR = PWM;

    PWML = min(254, PWML);
    wr.PWML = uint8(max(0, PWML));
    PWMR = min(254, PWMR);
    wr.PWMR = uint8(max(0, PWMR));
end