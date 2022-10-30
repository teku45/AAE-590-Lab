function [b1] = blimp_control_spd(b1, time)
% setting the PWM limits 
    b1.DIRL = 1;
    b1.DIRR = 1;
    
    
    PWML = 0;
    PWMR = 0;
    PWMV = 0;

%     Kp = 0.0305;
%     Ki = 0.006722;
%     Kd = 0.00047;
    
    %v_curr = pdist([b1.pos; b1.pos_old]) / time.dt
    v_curr = sqrt((b1.pos(2)-b1.pos_old(2))^2 + (b1.pos(1)-b1.pos_old(1))^2) / time.dt;
    b1.espd = b1.forward_spd - v_curr;
    %[b1.espd]
    espd_deriv = (b1.espd -  b1.espd_old) / time.dt;
    b1.espd_cum = b1.espd_cum + b1.espd;
    PWM = Kp * b1.espd + Ki * b1.espd_cum + Kd * espd_deriv;
    % Food for thought: is this the error to be added to v_curr or just the
    % new wheel velocity 

    % Update old error
    b1.espd_old = b1.espd;

    PWML = PWM;
    PWMR = PWM;

    if isnan(b1.espd_cum)
        b1.espd_cum=0;
    end

    b1.PWMV = min(250, b1.PWMV);
    b1.PWMV = uint8(max(0, b1.PWMV));
    b1.PWML = min(250, b1.PWML);
    b1.PWML = uint8(max(0, b1.PWML));
    b1.PWMR = min(250, b1.PWMR);
    b1.PWMR = uint8(max(0, b1.PWMR));

end



