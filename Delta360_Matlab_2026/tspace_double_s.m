function tspace_traj_ds = tspace_double_s(points, v_max, a_max, j_max)

    % number of segments
    num_points = size(points, 1);
    num_segments = num_points - 1;
    
    % init arrays
    t_sim_ts = 0; 
    a_cart_sim_ts = [0, 0, 0];
    
    current_time = 0;
    
    for k = 1:num_segments
        P_start = points(k, :);
        P_end   = points(k+1, :);
        delta_P = P_end - P_start;
        
        D = norm(delta_P);
        u = delta_P / D;

        % max vel and acc
        if v_max >= (a_max^2 / j_max)
            a_pk_v = a_max;
            tj_v = a_max / j_max;
            ta_v = (v_max / a_max) - tj_v;
        else
            a_pk_v = sqrt(v_max * j_max);
            tj_v = sqrt(v_max / j_max);
            ta_v = 0;
        end

        % dist to reach max vel and then 0
        D_reach = v_max * (2*tj_v + ta_v);

        if D >= D_reach
            % reaches v_max
            a_pk = a_pk_v;
            tj = tj_v;
            ta = ta_v;
            tv = (D - D_reach) / v_max;
        else
            % not reach v_max
            tv = 0;
            D_amax = 2 * (a_max^3 / j_max^2);
            
            if D >= D_amax
                % reache a_max not v_max
                a_pk = a_max;
                tj = a_max / j_max;
                % solve for t
                x = (-tj + sqrt(tj^2 + 4 * D / a_max)) / 2;
                ta = x - tj;
            else
                % not reach max a or max v
                ta = 0;
                tj = (D / (2 * j_max))^(1/3);
                a_pk = j_max * tj;
            end
        end

        % time points
        t1 = tj;
        t2 = tj + ta;
        t3 = 2*tj + ta;
        t4 = 2*tj + ta + tv;
        t5 = 3*tj + ta + tv;
        t6 = 3*tj + 2*ta + tv;
        T  = 4*tj + 2*ta + tv;

        t_seg = current_time + [0; t1; t2; t3; t4; t5; t6; T];
        a_seg = [0; a_pk; a_pk; 0; 0; -a_pk; -a_pk; 0];
        
        % project
        a_seg = a_seg * u; 

        % append to traj
        t_sim_ts      = [t_sim_ts; t_seg(2:end)];
        a_cart_sim_ts = [a_cart_sim_ts; a_seg(2:end, :)];

        current_time = current_time + T;
    end

    tspace_traj_ds = timeseries(a_cart_sim_ts, t_sim_ts);
end