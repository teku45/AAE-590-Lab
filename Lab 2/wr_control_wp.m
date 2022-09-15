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
    desired_angle = atan2d(target_wayP(2) - wr.pos(2), target_wayP(1) - wr.pos(1));
    actual_angle = atan2d(wr.heading_vec(2), wr.heading_vec(1));
    wr.heading_dir = [cos(desired_angle), sin(desired_angle)];
    e_heading = desired_angle - actual_angle;

    % Call heading func if the error is outside the error margin
    % Call speed func if error is outside error margin
    if (e_heading > 15)
        wr = wr_control_heading(wr, time);
    elseif (pdist([wr.pos; target_wayP]) > 100) 
        wr = wr_control_spd(wr, time);
    else
        wr.curWP = wr.curWP + 1;
    end

end