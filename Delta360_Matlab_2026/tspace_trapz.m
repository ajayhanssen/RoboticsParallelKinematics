% robot params
l = 400e-3;
rb = 222.105421e-3;
rm = 42e-3;

% motion limits
v_max = 40e-3;
a_max = 70e-3;
j_max = 5e-3;

points = [0.0, 0.0, 0.4; 0.1,0.1,0.4; -0.1,-0.1,0.4; 0.1,-0.1, 0.4; 0.0, 0.0, 0.4];

% number of segments
num_points = size(points, 1);
num_segments = num_points - 1;

% init arrays
t_sim_ts = 0; 
v_cart_sim_ts = [0, 0, 0]; 

current_time = 0;

for k = 1:num_segments
    P_start = points(k, :);
    P_end   = points(k+1, :);
    delta_P = P_end - P_start;

    % straight line dist
    D = norm(delta_P);

    % direction vec
    u = delta_P / D;

    % calc trapezoidal or triangular
    if D >= (v_max^2 / a_max)
        % trapezoidal
        ta = v_max / a_max;
        tc = (D - (v_max^2 / a_max)) / v_max;
        T = 2*ta + tc;
        v_pk = v_max;
    else
        % triangular
        ta = sqrt(D / a_max);
        tc = 0;
        T = 2*ta;
        v_pk = a_max * ta;
    end

    % peak v to direction
    v_peak_vec = v_pk * u;

    % times and vels
    if tc > 1e-6
        % trapezoidal (start, peak start, peak end, stop)
        t_seg = current_time + [0; ta; ta + tc; T];
        v_seg = [zeros(1,3); v_peak_vec; v_peak_vec; zeros(1,3)];
    else
        % triangular (start, peak, stop)
        t_seg = current_time + [0; ta; T];
        v_seg = [zeros(1,3); v_peak_vec; zeros(1,3)];
    end

    % add to traj array
    t_sim_ts = [t_sim_ts; t_seg(2:end)];
    v_cart_sim_ts = [v_cart_sim_ts; v_seg(2:end, :)];

    current_time = current_time + T;
end
