% robot params
l = 400e-3;
rb = 222.105421e-3;
rm = 42e-3;

% motion limits
v_max = 50e-3;
a_max = 100e-3;
j_max = 5e-3;
v_max_op = v_max/2;

points = [0.0, 0.0, 0.4; 0.0,0.1,0.45; 0.1,-0.1,0.42; -0.1,0.0, 0.45; 0.0, 0.0, 0.4];

% joint space
joints = zeros(size(points));
for i = 1:size(points,1)
    [s1, s2, s3] = inverse_kinematics(points(i,1), points(i,2), points(i,3), rb, rm, l);
    joints(i,:) = [s1, s2, s3];
end

% number of segments
num_points = size(joints, 1);
num_segments = num_points - 1;

v_via = zeros(num_points, 3);
for k = 2:num_segments
    D_prev = joints(k, :) - joints(k-1, :);
    D_next = joints(k+1, :) - joints(k, :);
    for j = 1:3
        % If the joint continues in the same direction, pass without stopping
        if sign(D_prev(j)) == sign(D_next(j))
            % Limit via-velocity to ensure it is kinematically reachable within the segment distance
            max_v_prev = sqrt(a_max * abs(D_prev(j)));
            max_v_next = sqrt(a_max * abs(D_next(j)));
            v_via(k, j) = sign(D_prev(j)) * min([v_max_op, max_v_prev, max_v_next]);
        else
            % Must stop to safely reverse direction
            v_via(k, j) = 0; 
        end
    end
end

% --- 2. Generate Synchronized Continuous Trajectory ---
t_sim_js_con = 0; 
v_sim_js_con = [0, 0, 0]; 
current_time = 0;

for k = 1:num_segments
    q_start = joints(k, :);
    q_end   = joints(k+1, :);

    % Fetch the boundary velocities we calculated
    v_start = v_via(k, :);
    v_end   = v_via(k+1, :);
    delta_q = q_end - q_start;

    % Step A: Calculate minimum required time (T_j) for each joint independently
    T_j = zeros(1, 3);
    for j = 1:3
        D = delta_q(j);
        if D == 0
            continue;
        end

        dir = sign(D);
        v0 = v_start(j);
        v1 = v_end(j);

        v_c = dir * v_max_op;
        a_c = dir * a_max;

        % Distances spent accelerating and decelerating
        d_acc = (v_c^2 - v0^2) / (2 * a_c);
        d_dec = (v_c^2 - v1^2) / (2 * a_c);

        if abs(D) >= abs(d_acc + d_dec)
            % Trapezoidal Profile (reaches cruise speed)
            t_a = (v_c - v0) / a_c;
            t_d = (v1 - v_c) / (-a_c); 
            t_c = (D - (d_acc + d_dec)) / v_c;
            T_j(j) = t_a + t_c + t_d;
        else
            % Triangular Profile (segment too short to reach full cruise speed)
            v_peak = dir * sqrt(a_max * abs(D) + 0.5 * (v0^2 + v1^2));
            t_a = (v_peak - v0) / a_c;
            t_d = (v1 - v_peak) / (-a_c);
            T_j(j) = t_a + t_d;
        end
    end

    % Step B: Synchronize to the slowest joint
    T_max = max(T_j);

    % Define a universal ramp time for this segment's synchronization
    t_ramp = v_max_op / a_max; 
    if T_max < 2 * t_ramp
        t_ramp = T_max / 2; % Fallback for short triangular segments
    end

    % Calculate the synchronized cruise velocity required to cover the exact distance in T_max
    % Formula derived from Area = delta_q using trapezoidal integration
    v_sync = (delta_q - 0.5 * t_ramp * (v_start + v_end)) / (T_max - t_ramp);

    % Step C: Generate corner points for interpolation
    if (T_max - 2*t_ramp) > 1e-6
        % Trapezoidal (4 timing points)
        t_seg = current_time + [0; t_ramp; T_max - t_ramp; T_max];
        v_seg = [v_start; v_sync; v_sync; v_end];
    else
        % Triangular (3 timing points)
        t_seg = current_time + [0; t_ramp; T_max];
        v_seg = [v_start; v_sync; v_end];
    end

    % Append to simulation arrays (skipping the t=0 overlap)
    t_sim_js_con = [t_sim_js_con; t_seg(2:end)];
    v_sim_js_con = [v_sim_js_con; v_seg(2:end, :)];

    current_time = current_time + T_max;
end