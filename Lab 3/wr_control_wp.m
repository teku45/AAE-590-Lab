function [wr] = wr_control_wp(wr, time, obs_pos)
% default values
%[obs_pos(1) obs_pos(2) obs_pos(3)]

if obs_pos(3) > 35
    wr.obs_mode = 1;
else
    wr.obs_mode = 0;
end

if (wr.obs_mode == 0 && wr.obs_prev_mode == 1)
    Waypoints = RRTsimulator_AAE590(wr.pos, obs_pos);
    wr.WP = Waypoints(:,1:2);
    wr.curWP = 1;
end

wr.obs_prev_mode = wr.obs_mode;

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
elseif (sqrt((wr.pos(2)-target_wayP(2))^2 + (wr.pos(1)-target_wayP(1))^2) > 100)
    wr = wr_control_spd(wr, time);
    fprintf("speed mode");
else
    wr.curWP = wr.curWP + 1;
    wr.espd_cum = 0;
    wr.e_heading_cum = 0;
end
    wr.PWML = (1-wr.obs_mode);
    wr.PWMR = (1-wr.obs_mode);


end