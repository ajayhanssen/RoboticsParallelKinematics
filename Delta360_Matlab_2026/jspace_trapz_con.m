% robot params
l = 400e-3;
rb = 222.105421e-3;
rm = 42e-3;

% motion limits
v_max = 50e-3;
a_max = 100e-3;
j_max = 5e-3;
v_op = v_max/2;

P1 = [0.0, 0.1, 0.4];
P2 = [0.075, -0.075, 0.45];
P3 = [-0.05, -0.05, 0.4];
P1_2 = (P1 + P2)/2;
P2_3 = (P2 + P3)/2;
points = [P1; P2; P3];
points = [P1; P1_2; P2; P2_3; P3];

% joint space
joints = zeros(size(points));
for i = 1:size(points,1)
    [s1, s2, s3] = inverse_kinematics(points(i,1), points(i,2), points(i,3), rb, rm, l);
    joints(i,:) = [s1, s2, s3];
end

% number of segments
num_points = size(joints, 1);
num_segments = num_points - 1;

T_seg = zeros(num_segments, 1);
for k = 1:num_segments
    % find the joint that has to move the furthest
    max_joint_displacement = max(abs(joints(k+1, :) - joints(k, :)));
    % required time
    T_seg(k) = max_joint_displacement / v_op; 
end

t_way = [0; cumsum(T_seg)];

% central diff crit
v_joints = zeros(num_points, 3);
% vel at first and last is 0
for i = 2:(num_points - 1)
    v_joints(i, :) = (joints(i+1, :) - joints(i-1, :)) / (t_way(i+1) - t_way(i-1));
end

% jspace traj
dt = 0.01;
t_sim_js_con = [];
q_sim_js_con = [];
qd_sim_js_con = [];

for k = 1:num_segments
    q0 = joints(k, :);
    qf = joints(k+1, :);
    v0 = v_joints(k, :);
    vf = v_joints(k+1, :);
    T = T_seg(k);
    
    t_vec = (0:dt:T)';
    
    q_seg = zeros(length(t_vec), 3);
    qd_seg = zeros(length(t_vec), 3);
    
    for j = 1:3
        % cubic coefficients
        a0 = q0(j);
        a1 = v0(j);
        a2 = (3*(qf(j) - q0(j)) / T^2) - ((2*v0(j) + vf(j)) / T);
        a3 = (-2*(qf(j) - q0(j)) / T^3) + ((v0(j) + vf(j)) / T^2);
        
        % Position and velocity
        q_seg(:, j) = a0 + a1*t_vec + a2*t_vec.^2 + a3*t_vec.^3;
        qd_seg(:, j) = a1 + 2*a2*t_vec + 3*a3*t_vec.^2;
    end
    
    % append to total trajectory
    if k == 1
        t_sim_js_con = [t_sim_js_con; t_vec];
        q_sim_js_con = [q_sim_js_con; q_seg];
        qd_sim_js_con = [qd_sim_js_con; qd_seg];
    else
        t_sim_js_con = [t_sim_js_con; t_vec(2:end) + t_way(k)];
        q_sim_js_con = [q_sim_js_con; q_seg(2:end, :)];
        qd_sim_js_con = [qd_sim_js_con; qd_seg(2:end, :)];
    end
end