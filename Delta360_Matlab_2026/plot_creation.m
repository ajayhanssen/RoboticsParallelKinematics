fsize = 20;
lwidth = 2;

%% forward kinematics plot
outvar = out.fw_error;
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
legend("$x_\mathrm{MB}$", "$y_\mathrm{MB}$", "$z_\mathrm{MB}$", 'Interpreter', 'latex', 'Location', 'west', ...
    'FontSize', fsize)
xlabel('time in s', 'Interpreter', 'latex', 'FontSize', fsize)
ylabel('position MB in m', 'Interpreter', 'latex', 'FontSize', fsize)

nexttile(3)
plot(t, s1_IK, t, s2_IK, t, s3_IK, 'LineWidth', lwidth)
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