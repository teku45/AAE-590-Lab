function [b1] = blimp_control_heading(b1, time)

   % default values
    b1.DIRL = 1;
    b1.DIRR = 1;
    PWML = 0;
    PWMR = 0;
    PWMV = 0;

    Kp = 1.48305;
    Kd = 0.015722;
    Ki = 0.0247;

    % POTENTIALLY SWITCH SIGNS
    desired_angle = atan2d(b1.heading_dir(2), b1.heading_dir(1));% + 180;
    actual_angle = atan2d(b1.heading_vec(2), b1.heading_vec(1));
    b1.e_heading = desired_angle - actual_angle;
  

    % adjusting heading error to [-180, 180]
    if (b1.e_heading > 180)
        b1.e_heading = b1.e_heading - 360;
    end

    % account for error margin
    if (abs(b1.e_heading) < 15)
        b1.e_heading = 0;
    end

    e_heading_deriv = (b1.e_heading - b1.e_heading_old) / time.dt;
    b1.e_heading_cum = b1.e_heading_cum + b1.e_heading;
    PWM = Kp * b1.e_heading + Ki * b1.e_heading_cum + Kd * e_heading_deriv;
    if (abs(b1.e_heading) < 15)
        PWM = 0;
    end
    % Update old error
    b1.e_heading_old = b1.e_heading;

    if (b1.e_heading < 0) % too far to the left
        b1.DIRL = 1;
        b1.DIRR = 0;
    else % too far to the right
        b1.DIRL = 0;
        b1.DIRR = 1;
    end

    PWML = abs(PWM);
    PWMR = abs(PWM);

    % setting the PWM limits, keep the codes here

%     scatter(time.curr,actual_angle,'g'); hold on;
%     scatter(time.curr,desired_angle,'r');
%     drawnow;

% setting the PWM limits 
b1.PWMV = min(250, b1.PWMV);
b1.PWMV = uint8(max(0, b1.PWMV));
b1.PWML = min(250, b1.PWML);
b1.PWML = uint8(max(0, b1.PWML));
b1.PWMR = min(250, b1.PWMR);
b1.PWMR = uint8(max(0, b1.PWMR));

end


