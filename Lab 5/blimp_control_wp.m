function [b1] = blimp_control_wp(b1, time)

   % default values
    b1.DIRL = 1;
    b1.DIRR = 1;
    b1.DIRV = 1;
    
    PWML = 0;
    PWMR = 0;
    PWMV = 0;
    
    % Exit function if all waypoints are reached
    N = length(b1.WP);
    if (b1.curWP > N)
        return;
    end

    % Coordinates of current target waypoint based on its current index
    target_wayP = b1.WP(b1.curWP,:);

    % Heading angle
    desired_angle = atan2d((target_wayP(2) - b1.pos(2)),( target_wayP(1) - b1.pos(1)));
    
    %actual_angle = atan2d(b1.heading_vec(2), b1.heading_vec(1));
    
    b1.heading_dir = [cosd(desired_angle), sind(desired_angle)];
    %e_heading = desired_angle - actual_angle;
    %[actual_angle desired_angle e_heading]
    % Call heading func if the error is outside the error margin
    % Call speed func if error is outside error margin
   
    
    %if (((abs(e_heading)) > 15 && (abs(e_heading) < 180)) || ((abs(e_heading) < 345 && abs(e_heading) > 180)))
    
    %desired_angle = atan2d(b1.heading_dir(2), b1.heading_dir(1));% + 180;
    actual_angle = atan2d(b1.heading_vec(2), b1.heading_vec(1));
    b1.e_heading = desired_angle - actual_angle;
    [actual_angle, b1.e_heading]

    % adjusting heading error to [-180, 180]
    if (b1.e_heading > 180)
        b1.e_heading = b1.e_heading - 360;
    end

    if (abs(b1.evertical > 20))
        b1 = blimp_control_vertical(b1, time);
        
        fprintf("vertical mode");
    elseif (abs(b1.e_heading) > 15)
        b1 = blimp_control_heading(b1, time);
       
        fprintf("heading mode");
    elseif (sqrt((b1.pos(2)-target_wayP(2))^2 + (b1.pos(1)-target_wayP(1))^2) > 100) 
        
        b1 = blimp_control_spd(b1, time);
        fprintf("speed mode");
    else
        b1.curWP = b1.curWP + 1;
        b1.espd_cum = 0;
        b1.e_heading_cum = 0;
    end
    b1.curWP



b1.PWML = min(254, b1.PWML);
b1.PWML = uint8(max(0, b1.PWML));
b1.PWMR = min(254, b1.PWMR);
b1.PWMR = uint8(max(0, b1.PWMR));
b1.PWMV = min(254, b1.PWMV);
b1.PWMV = uint8(max(0, b1.PWMV));

end