% robot params
l = 400e-3;
rb = 222.105421e-3;
rm = 42e-3;

% motion limits joint space
v_max_j = 50e-3;
a_max_j = 100e-3;
j_max_j = 5;

% motion limits task space
v_max_t = 40e-3;
a_max_t = 70e-3;
j_max_t = 5;

% waypoints
P1 = [0.0, 0.1, 0.4];
P2 = [0.075, -0.075, 0.45];
P3 = [-0.05, -0.05, 0.4];
points = [P1; P2; P3];

% trapezoidal and double s trajectories
[jspace_traj, joints] = jspace_trapz(points, v_max_j, a_max_j, l, rb, rm);
[jspace_traj_ds, joints] = jspace_double_s(points, v_max_j, a_max_j, j_max_j, l, rb, rm);

tspace_traj = tspace_trapz(points, v_max_t, a_max_t);
tspace_traj_ds = tspace_double_s(points, v_max_t, a_max_t, j_max_t);

% continuous trajectories
% extra via points
P1_2 = (P1 + P2)/2;
P2_3 = (P2 + P3)/2;
points = [P1; P1_2; P2; P2_3; P3];

jspace_traj_cnt = jspace_continuous(points, v_max_j, a_max_j, l, rb, rm);