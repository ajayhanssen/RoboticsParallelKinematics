% robot params
l = 400e-3;
rb = 222.105421e-3;
rm = 42e-3;

% motion limits
v_max = 50e-3;
a_max = 100e-3;
j_max = 5e-3;

P1 = [0.0, 0.1, 0.4];
P2 = [0.075, -0.075, 0.45];
P3 = [-0.05, -0.05, 0.4];
points = [P1; P2; P3];

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
v_sim_js = [0, 0, 0];

current_time = 0;

for k = 1:num_segments
    q_start = joints(k, :);
    q_end   = joints(k+1, :);
    delta_q = q_end - q_start;

    % calc time required for each joint
    T_j = zeros(1, 3);
    ta_j = zeros(1, 3);

    for j = 1:3
        D = abs(delta_q(j));
        if D == 0
            continue;
        end

        % trapezoidal or triangular
        if D >= (v_max^2 / a_max)
            % trapezoidal
            ta = v_max / a_max;
            tc = (D - (v_max^2 / a_max)) / v_max;
            T_j(j) = 2*ta + tc;
            ta_j(j) = ta;
        else
            % triangular
            ta = sqrt(D / a_max);
            T_j(j) = 2*ta;
            ta_j(j) = ta;
        end
    end

    % synchronize to slowest joint
    [T_max, slowest_idx] = max(T_j);

    ta_max = ta_j(slowest_idx);
    tc_max = T_max - 2*ta_max;

    % 3. Calculate synchronized velocity for this segment
    % Formula: D = v_sync * ta_max + v_sync * tc_max
    v_sync = delta_q / (ta_max + tc_max);

    % 4. Generate corner points for interpolation
    % We use corner points because Simulink will linearly interpolate between them
    if tc_max > 1e-6
        % Trapezoidal (4 points: start, end of accel, start of decel, end)
        t_seg = current_time + [0; ta_max; ta_max + tc_max; T_max];
        v_seg = [zeros(1,3); v_sync; v_sync; zeros(1,3)];
    else
        % Triangular (3 points: start, peak, end)
        t_seg = current_time + [0; ta_max; T_max];
        v_seg = [zeros(1,3); v_sync; zeros(1,3)];
    end

    % add to traj array
    t_sim_js = [t_sim_js; t_seg(2:end)];
    v_sim_js = [v_sim_js; v_seg(2:end, :)];

    current_time = current_time + T_max;
end
