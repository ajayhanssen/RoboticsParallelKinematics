function [jspace_traj_ds, joints] = jspace_double_s(points, v_max, a_max, j_max, l, rb, rm)

    % joint space
    joints = zeros(size(points));
    for i = 1:size(points,1)
        [s1, s2, s3] = inverse_kinematics(points(i,1), points(i,2), points(i,3), rb, rm, l);
        joints(i,:) = [s1, s2, s3];
    end
    
    % number of segments
    num_points = size(joints, 1);
    num_segments = num_points - 1;
    
    % init arrays
    t_sim_js = 0;
    a_sim_js = [0, 0, 0];
    
    current_time = 0;
    
    for k = 1:num_segments
        q_start = joints(k, :);
        q_end   = joints(k+1, :);
        delta_q = q_end - q_start;
        
        D_max = max(abs(delta_q));
        if D_max == 0
            continue;
        end
        
        % 1. Calculate timings
        v_reach_a = (a_max^2) / j_max; 
        
        if v_max < v_reach_a
            tj = sqrt(v_max / j_max);
            ta = 0;
        else
            tj = a_max / j_max;
            ta = (v_max - v_reach_a) / a_max;
        end
        
        T_acc = 2 * tj + ta; 
        D_acc = T_acc * v_max; 
        
        if D_max < D_acc
            p = (a_max^2) / j_max;
            v_peak = (-p + sqrt(p^2 + 4 * D_max * a_max)) / 2;
            
            if v_peak < v_reach_a
                v_peak = (D_max^2 * j_max / 4)^(1/3);
                tj = sqrt(v_peak / j_max);
                ta = 0;
            else
                tj = a_max / j_max;
                ta = (v_peak - v_reach_a) / a_max;
            end
            tv = 0;
        else
            tv = (D_max - D_acc) / v_max;
        end
        
        % time boundaries
        t1 = tj;
        t2 = tj + ta;
        t3 = 2*tj + ta;
        t4 = 2*tj + ta + tv;
        t5 = 3*tj + ta + tv;
        t6 = 3*tj + 2*ta + tv;
        t7 = 4*tj + 2*ta + tv;
        
        % peak acc
        a_peak = j_max * tj;
        
        % corner points
        t_corners = current_time + [0; t1; t2; t3; t4; t5; t6; t7];
        a_1D      = [0; a_peak; a_peak; 0; 0; -a_peak; -a_peak; 0];
        
        % delete duplicate times
        [t_corners_unique, unique_idx] = unique(t_corners);
        a_1D_unique = a_1D(unique_idx);
        
        % scaling
        a_seg_joints = a_1D_unique * (delta_q / D_max);
        
        % append to traj
        if isempty(t_sim_js)
            t_sim_js = t_corners_unique;
            a_sim_js = a_seg_joints;
        else
            % skip first point
            t_sim_js = [t_sim_js; t_corners_unique(2:end)];
            a_sim_js = [a_sim_js; a_seg_joints(2:end, :)];
        end
        
        current_time = current_time + t7;
    end
    
    jspace_traj_ds = timeseries(a_sim_js, t_sim_js);

end