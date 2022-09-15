function [wr] = wr_control_wp(wr, time)
   % default values
    wr.DIRL = 1;
    wr.DIRR = 1;
    PWML = 0;
    PWMR = 0;

    % Exit function if all waypoints are reached
    N = length(wr.WP);
    if (wr.curWP > N)
        return;
    end

    % Coordinates of current target waypoint based on its current index
    target_wayP = wr.WP(wr.curWP,:);

    % Heading angle
    desired_angle = atand(target_wayP(2) - wr.pos(2))/( target_wayP(1) - wr.pos(1)));
    actual_angle = atand((wr.heading_vec(2), wr.heading_vec(1));
    wr.heading_dir = [cosd(desired_angle), sind(desired_angle)];
    e_heading = desired_angle - actual_angle;
    [actual_angle desired_angle e_heading]
    % Call heading func if the error is outside the error margin
    % Call speed func if error is outside error margin
    if (e_heading > 15)
        wr = wr_control_heading(wr, time);
        %fprintf("heading mode");
    elseif (sqrt((wr.pos(2)-target_wayP(2))^2 + (wr.pos(1)-target_wayP(1))^2) > 100) 
        
        wr = wr_control_spd(wr, time);
        %fprintf("speed mode");
    else
        wr.curWP = wr.curWP + 1;
    end

end