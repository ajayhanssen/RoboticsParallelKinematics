fsize = 20;
lwidth = 2;

%% forward kinematics plot
outvar = out.fw_error;
t = outvar.Time;

error = outvar.Data(:,1);

x_MB = outvar.Data(:,2);
y_MB = outvar.Data(:,3);
z_MB = outvar.Data(:,4);

x_FK = outvar.Data(:,5);
y_FK = outvar.Data(:,6);
z_FK = outvar.Data(:,7);

% plot
f = tiledlayout(2,2);
f.TileSpacing = 'compact';

nexttile(1)
plot(t, x_MB, t, y_MB, t, z_MB, 'LineWidth', lwidth)
grid on
legend("$x_\mathrm{MB}$", "$y_\mathrm{MB}$", "$z_\mathrm{MB}$", 'Interpreter', 'latex', 'Location', 'west', ...
    'FontSize', fsize)
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('position MB in m', 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(3)
plot(t, x_FK, t, y_FK, t, z_FK, 'LineWidth', lwidth)
grid on
legend("$x_\mathrm{FK}$", "$y_\mathrm{FK}$", "$z_\mathrm{FK}$", 'Interpreter', 'latex', 'Location', 'west', ...
    'FontSize', fsize)
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('position FK in m', 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(2, [2,1])
plot(t, error, 'LineWidth', lwidth)
grid on
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('error in m', 'Interpreter', 'latex', 'FontSize', fsize)

%% inverse kinematics plot
outvar = out.ik_error;
t = outvar.Time;

error = outvar.Data(:,1);

s1_IN = outvar.Data(:,2);
s2_IN = outvar.Data(:,3);
s3_IN = outvar.Data(:,4);

s1_IK = outvar.Data(:,5);
s2_IK = outvar.Data(:,6);
s3_IK = outvar.Data(:,7);

% plot
f = tiledlayout(2,2);
f.TileSpacing = 'compact';

nexttile(1)
plot(t, s1_IN, t, s2_IN, t, s3_IN, 'LineWidth', lwidth)
grid on
legend("$s_\mathrm{1,IN}$", "$s_\mathrm{2,IN}$", "$s_\mathrm{3,IN}$", 'Interpreter', 'latex', 'Location', 'northwest', ...
    'FontSize', fsize)
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint pos. input in m', 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(3)
plot(t, s1_IK, t, s2_IK, t, s3_IK, 'LineWidth', lwidth)
grid on
legend("$s_\mathrm{1,IK}$", "$s_\mathrm{2,IK}$", "$s_\mathrm{3,IK}$", 'Interpreter', 'latex', 'Location', 'northwest', ...
    'FontSize', fsize)
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint pos. IK in m', 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(2, [2,1])
plot(t, error, 'LineWidth', lwidth)
grid on
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('error in m', 'Interpreter', 'latex', 'FontSize', fsize)

%% joint space trapz plots
outvar = out.joint_trapz;
t = outvar.Time;
t = t(t<t_sim_js(end));

s1_trapz = outvar.Data(t<t_sim_js(end),1);
s2_trapz = outvar.Data(t<t_sim_js(end),2);
s3_trapz = outvar.Data(t<t_sim_js(end),3);

v1_trapz = outvar.Data(t<t_sim_js(end),4);
v2_trapz = outvar.Data(t<t_sim_js(end),5);
v3_trapz = outvar.Data(t<t_sim_js(end),6);

a1_trapz = outvar.Data(t<t_sim_js(end),7);
a2_trapz = outvar.Data(t<t_sim_js(end),8);
a3_trapz = outvar.Data(t<t_sim_js(end),9);

x_MB = outvar.Data(t<t_sim_js(end),10);
y_MB = outvar.Data(t<t_sim_js(end),11);
z_MB = outvar.Data(t<t_sim_js(end),12);

figure
f = tiledlayout(3,1);
f.TileSpacing = 'compact';

% trapz trajectory
nexttile
plot(t, s1_trapz, t, s2_trapz, t, s3_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$s_\mathrm{1}$", "$s_\mathrm{2}$", "$s_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1636 0.7530 0.0548, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint pos. in m', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js(end)])
hold off

nexttile
plot(t, v1_trapz, t, v2_trapz, t, v3_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$v_\mathrm{1}$", "$v_\mathrm{2}$", "$v_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1628 0.4474 0.0558, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint vel. in $\mathrm{m\,s}^{-1}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js(end)])
hold off

nexttile
plot(t, a1_trapz, t, a2_trapz, t, a3_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$a_\mathrm{1}$", "$a_\mathrm{2}$", "$a_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1614 0.1466 0.0569, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint acc. in $\mathrm{m\,s}^{-2}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js(end)])
hold off

%exportgraphics(gcf,"../Report/cus_imgs/joint_trapz.pdf",ContentType="vector")

% xyz of robot
figure
plot(t, x_MB, t, y_MB, t, z_MB, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('position MB in m', 'Interpreter', 'latex', 'FontSize', fsize)
legend("$x_\mathrm{MB}$", "$y_\mathrm{MB}$", "$z_\mathrm{MB}$", 'Interpreter', 'latex', 'Location', 'west', ...
    'FontSize', fsize)

xlim([0 t_sim_js(end)])
hold off

%exportgraphics(gcf,"../Report/cus_imgs/joint_trapz_xyz.pdf",ContentType="vector")

%% task space trapz plots

outvar = out.task_trapz;
t = outvar.Time;
t = t(t<t_sim_ts(end));

x_trapz = outvar.Data(t<t_sim_ts(end),1);
y_trapz = outvar.Data(t<t_sim_ts(end),2);
z_trapz = outvar.Data(t<t_sim_ts(end),3);

xd_trapz = outvar.Data(t<t_sim_ts(end),4);
yd_trapz = outvar.Data(t<t_sim_ts(end),5);
zd_trapz = outvar.Data(t<t_sim_ts(end),6);

xdd_trapz = outvar.Data(t<t_sim_ts(end),7);
ydd_trapz = outvar.Data(t<t_sim_ts(end),8);
zdd_trapz = outvar.Data(t<t_sim_ts(end),9);

figure
f = tiledlayout(3,1);
f.TileSpacing = 'compact';

% trapz trajectory
nexttile
plot(t, x_trapz, t, y_trapz, t, z_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_ts, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$x$", "$y$", "$z$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1436 0.7530 0.0548, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('end-eff. pos. in m', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_ts(end)])
hold off

nexttile
plot(t, xd_trapz, t, yd_trapz, t, zd_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_ts, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$\dot{x}$", "$\dot{y}$", "$\dot{z}$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1428 0.4474 0.0558, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('end-eff. vel. in $\mathrm{m\,s}^{-1}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_ts(end)])
hold off

nexttile
plot(t, xdd_trapz, t, ydd_trapz, t, zdd_trapz, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_ts, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$\ddot{x}$", "$\ddot{y}$", "$\ddot{z}$"], "FontSize", 20, "Interpreter", "latex", "Location", "none", "Position", [0.1414 0.1466 0.0569, 0.1355])
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('end-eff. acc. in $\mathrm{m\,s}^{-2}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_ts(end)])
hold off

%exportgraphics(gcf,"../Report/cus_imgs/task_trapz.pdf",ContentType="vector")

%% joint vel, acc jspace vs tspace
% jspace data
outvar = out.joint_trapz;
t_js = outvar.Time;
t_js = t_js(t<t_sim_js(end));

v1_js = outvar.Data(t_js<t_sim_js(end),4);
v2_js = outvar.Data(t_js<t_sim_js(end),5);
v3_js = outvar.Data(t_js<t_sim_js(end),6);

a1_js = outvar.Data(t_js<t_sim_js(end),7);
a2_js = outvar.Data(t_js<t_sim_js(end),8);
a3_js = outvar.Data(t_js<t_sim_js(end),9);

% tspace data
outvar = out.task_trapz;
t_ts = outvar.Time;
t_ts = t_ts(t<t_sim_ts(end));

v1_ts = outvar.Data(t_ts<t_sim_ts(end),13);
v2_ts = outvar.Data(t_ts<t_sim_ts(end),14);
v3_ts = outvar.Data(t_ts<t_sim_ts(end),15);

a1_ts = outvar.Data(t_ts<t_sim_ts(end),16);
a2_ts = outvar.Data(t_ts<t_sim_ts(end),17);
a3_ts = outvar.Data(t_ts<t_sim_ts(end),18);

figure
f = tiledlayout(2,2);
f.TileSpacing = 'compact';

nexttile
plot(t_js, v1_js, t_js, v2_js, t_js, v3_js, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$v_\mathrm{1}$", "$v_\mathrm{2}$", "$v_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
ylabel('joint vel. in $\mathrm{m\,s}^{-1}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js(end)])
title("Joint Space", 'Interpreter', 'latex', 'FontSize', fsize)
hold off

nexttile
plot(t_ts, v1_ts, t_ts, v2_ts, t_ts, v3_ts, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_ts, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$v_\mathrm{1}$", "$v_\mathrm{2}$", "$v_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
xlim([0 t_sim_ts(end)])
title("Task Space", 'Interpreter', 'latex', 'FontSize', fsize)
hold off

nexttile
plot(t_js, a1_js, t_js, a2_js, t_js, a3_js, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_js, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$a_\mathrm{1}$", "$a_\mathrm{2}$", "$a_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint acc. in $\mathrm{m\,s}^{-2}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js(end)])
hold off

nexttile
plot(t_ts, a1_ts, t_ts, a2_ts, t_ts, a3_ts, 'LineWidth', lwidth)
grid on; hold on
xline(t_sim_ts, 'LineStyle', ':', 'LineWidth', lwidth, 'Color', [0.8, 0.8, 0.8])
legend(["$a_\mathrm{1}$", "$a_\mathrm{2}$", "$a_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_ts(end)])
hold off

exportgraphics(gcf,"../Report/cus_imgs/joint_vs_task.pdf",ContentType="vector")

%%
figure
plot3(x_MB, y_MB, z_MB, 'LineWidth', lwidth)
hold on
plot3(x_trapz, y_trapz, z_trapz, 'LineWidth', lwidth)
plot3(points(:,1), points(:,2), points(:,3), 'Color', [0,0,0], 'Marker','o', 'MarkerSize', 10)
grid on
legend(["joint space", "task space"], "FontSize", 20, "Interpreter", "latex", "Location","northwest")
xlabel('x in m', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('y in m', 'Interpreter', 'latex', 'FontSize', fsize)
zlabel('z in m', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([min(x_MB)-0.025 max(x_MB)+0.025])
ylim([min(y_MB)-0.025 max(y_MB)+0.025])
hold off

%exportgraphics(gcf,"../Report/cus_imgs/joint_vs_task_motion_3d.pdf",ContentType="vector")

%% continuous traj

outvar = out.joint_con;
t = outvar.Time;
t = t(t<t_sim_js_con(end));

s1_jcon = outvar.Data(t<t_sim_js_con(end),1);
s2_jcon = outvar.Data(t<t_sim_js_con(end),2);
s3_jcon = outvar.Data(t<t_sim_js_con(end),3);

v1_jcon = outvar.Data(t<t_sim_js_con(end),4);
v2_jcon = outvar.Data(t<t_sim_js_con(end),5);
v3_jcon = outvar.Data(t<t_sim_js_con(end),6);

x_MB_con = outvar.Data(t<t_sim_js_con(end),7);
y_MB_con = outvar.Data(t<t_sim_js_con(end),8);
z_MB_con = outvar.Data(t<t_sim_js_con(end),9);

figure
f = tiledlayout(2,2);
f.TileSpacing = 'compact';

% trapz trajectory
nexttile(2)
plot(t, s1_jcon, t, s2_jcon, t, s3_jcon, 'LineWidth', lwidth)
grid on; hold on
legend(["$s_\mathrm{1}$", "$s_\mathrm{2}$", "$s_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint pos. in m', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js_con(end)])
hold off
title("Without extra via-points", 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(4)
plot(t, v1_jcon, t, v2_jcon, t, v3_jcon, 'LineWidth', lwidth)
grid on; hold on
legend(["$v_\mathrm{1}$", "$v_\mathrm{2}$", "$v_\mathrm{3}$"], "FontSize", 20, "Interpreter", "latex")
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('joint vel. in $\mathrm{m\,s}^{-1}$', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([0 t_sim_js_con(end)])
hold off
title("With extra via-points", 'Interpreter', 'latex', 'FontSize', fsize)

%exportgraphics(gcf,"../Report/cus_imgs/joint_con.pdf",ContentType="vector")

%% straight line
figure
plot(x_MB_con, y_MB_con, 'LineWidth', lwidth)
hold on
plot(points(:,1), points(:,2), 'LineWidth', lwidth, 'Marker', 'o', 'MarkerSize', 10)
%plot(points(:,1), points(:,2), 'LineWidth', lwidth, 'Marker', 'o', 'MarkerSize', 10, 'Color', [0,0,0])
grid on
legend(["no extra via-pts.", "extra via-pts.", "straight line"], "FontSize", 20, "Interpreter", "latex", "Location","northwest")
xlabel('x in m', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('y in m', 'Interpreter', 'latex', 'FontSize', fsize)
xlim([min(x_MB)-0.025 max(x_MB)+0.025])
ylim([min(y_MB)-0.025 max(y_MB)+0.025])
hold off

exportgraphics(gcf,"../Report/cus_imgs/con_viapoints.pdf",ContentType="vector")