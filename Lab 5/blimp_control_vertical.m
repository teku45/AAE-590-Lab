function [b1] = blimp_control_vertical(b1, time)
% vertical control 

    b1.DIRL = 1;
    b1.DIRR = 1;
    PWML = 0;
    PWMR = 0;
    PWMV = 0;

    Kp = 1.48305;
    Kd = 0.015722;
    Ki = 0.0247;

    % POTENTIALLY SWITCH SIGNS
    desired_vertical = b1.des_vertical;
    actual_vertical = b1.pos(3);
    b1.evertical = desired_vertical - actual_vertical;
  

    % account for error margin
    if (abs(b1.evertical) < 5)
        b1.evertical = 0;
    end

    e_vertical_deriv = (b1.evertical - b1.evertical_old) / time.dt;
    b1.evertical_cum = b1.evertical_cum + b1.evertical;
    PWM = Kp * b1.evertical + Ki * b1.evertical_cum + Kd * e_vertical_deriv;
    if (abs(b1.evertical) < 15)
        PWM = 0;
    end
    % Update old error
    b1.evertical_old = b1.evertical;

    if (b1.evertical < 0) % too far to the left
        b1.DIRV = 1;
        b1.DIRV = 1;
    else % too far to the right
        b1.DIRV = 0;
        b1.DIRV = 1;
    end

    PWMV = abs(PWM);

  
%     scatter(time.curr,actual_angle,'g'); hold on;
%     scatter(time.curr,desired_angle,'r');
%     drawnow;

        % setting the PWM limits, keep the codes here

b1.PWMV = min(254, b1.PWMV);
b1.PWMV = uint8(max(0, b1.PWMV));
b1.PWML = min(254, b1.PWML);
b1.PWML = uint8(max(0, b1.PWML));
b1.PWMR = min(254, b1.PWMR);
b1.PWMR = uint8(max(0, b1.PWMR));


end
