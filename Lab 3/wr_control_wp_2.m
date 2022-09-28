function [wr] = wr_control_wp(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML = 0;
    PWMR = 0;

    % Coordinates of current target waypoint based on its current index
    target_wayP = wr.WP(wr.curWP,:);
    distance_to_target = target_wayP - wr.pos;

    % Heading angle
    desired_angle = atan2d((target_wayP(2) - wr.pos(2)),( target_wayP(1) - wr.pos(1)));
    
    wr.heading_dir = [cosd(desired_angle), sind(desired_angle)];

    % Call heading func if the error is outside the error margin
    % Call speed func if error is outside error margin
   
    %desired_angle = atan2d(wr.heading_dir(2), wr.heading_dir(1));% + 180;
    actual_angle = atan2d(wr.heading_vec(2), wr.heading_vec(1));
    wr.e_heading = desired_angle - actual_angle;
    %[actual_angle, wr.e_heading]

    % adjusting heading error to [-180, 180]
    if (wr.e_heading > 180)
        wr.e_heading = wr.e_heading - 360;
    end

    if (abs(wr.e_heading) > 15)
        wr = wr_control_heading(wr, time);
       
        fprintf("heading mode");
    elseif norm(distance_to_target) > 100 
        
        wr = wr_control_spd(wr, time);
        fprintf("speed mode");
    elseif wr.curWP < length(wr.WP(:,1))
        wr.curWP = wr.curWP + 1;
        wr.espd_cum = 0;
        wr.e_heading_cum = 0;
    else 
        wr.DIRL = 1;
        wr.DIRR = 1;
        wr.PWML = 0;
        wr.PWMR = 0;
    end

end