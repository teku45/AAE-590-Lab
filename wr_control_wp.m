function [wr] = wr_control_wp(wr, time)
    %% Error Calculation
    % Distance
    wr.heading_des = wr.WP(wr.curWP,:) - wr.pos;
    
    % Heading
    cur_ang_to_X = acosd(wr.heading_vec(1) / norm(wr.heading_vec));
    if wr.heading_vec(2) < 0
        cur_ang_to_X = 360 - cur_ang_to_X;
    end
    des_ang_to_X = acosd(wr.heading_des(1) / norm(wr.heading_des));
    if wr.heading_des(2) < 0
        des_ang_to_X = 360 - des_ang_to_X;
    end
    wr.head_err = cur_ang_to_X - des_ang_to_X;
    
    % Speed
    cur_spd = norm((wr.pos - wr.pos_old) / wr.dt);
    wr.spd_err = wr.forward_spd - cur_spd;
    
    %% Controller Decision
    if abs(wr.head_err) > wr.fwd_deg % Turn towards waypoint
        wr = wr_control_heading(wr, time);
    elseif norm(wr.heading_des) > wr.dist_mar % Move forward
        wr = wr_control_spd(wr, time);
    elseif wr.curWP < length(wr.WP(:,1)) % Increment waypoint
        wr.curWP = wr.curWP + 1;
    else % Final waypoint reached
        wr.DIRL = 1;
        wr.DIRR = 1;
        wr.PWML = 0;
        wr.PWMR = 0;
    end
    
    %% Updating Old Values
    wr.e_heading_old = wr.head_err;
    wr.espd_old = wr.spd_err;
end