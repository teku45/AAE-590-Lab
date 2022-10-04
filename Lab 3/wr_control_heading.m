function [wr] = wr_control_heading(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML = 0;
    PWMR = 0;

%     Kp = 12.838;
%     Ki = 3.431;
%     Kd = 10.671;

%     Kp = .78305;
%     Kd = 0.009722;
%     Ki = 0.00547;
    Kp = 1.48305;
    Kd = 0.015722;
    Ki = 0.0247;

    % POTENTIALLY SWITCH SIGNS
    desired_angle = atan2d(wr.heading_dir(2), wr.heading_dir(1));% + 180;
    actual_angle = atan2d(wr.heading_vec(2), wr.heading_vec(1));
    wr.e_heading = desired_angle - actual_angle;
  

    % adjusting heading error to [-180, 180]
    if (wr.e_heading > 180)
        wr.e_heading = wr.e_heading - 360;
    end

    % account for error margin
    if (abs(wr.e_heading) < 7.5)
        wr.e_heading = 0;
    end

    e_heading_deriv = (wr.e_heading - wr.e_heading_old) / time.dt;
    wr.e_heading_cum = wr.e_heading_cum + wr.e_heading*(1-wr.obs_mode);
    PWM = Kp * wr.e_heading + Ki * wr.e_heading_cum + Kd * e_heading_deriv;
    if (abs(wr.e_heading) < 15)
        PWM = 0;
    end
    % Update old error
    wr.e_heading_old = wr.e_heading;

    if (wr.e_heading < 0) % too far to the left
        wr.DIRL = 1;
        wr.DIRR = 0;
    else % too far to the right
        wr.DIRL = 0;
        wr.DIRR = 1;
    end

    PWML = abs(PWM)*(1-wr.obs_mode);
    PWMR = abs(PWM)*(1-wr.obs_mode);

    % setting the PWM limits, keep the codes here
    %PWML = min(90, PWML);
    PWML = rescale(PWML,30,90,'InputMin',0,'InputMax',255);
    wr.PWML = uint8(max(0, PWML));
    %PWMR = min(90, PWMR);
    PWMR = rescale(PWMR,30,90,'InputMin',0,'InputMax',255);
    wr.PWMR = uint8(max(0, PWMR));

    scatter(time.curr,actual_angle,'g'); hold on;
    scatter(time.curr,desired_angle,'r');
    drawnow;
    
end