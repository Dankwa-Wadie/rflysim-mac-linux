function ok = rflymac_quicktest(doPlot)
%RFLYMAC_QUICKTEST Fly the example controller against coptersim from a MATLAB script.
%   ok = rflymac_quicktest       runs 15 s (climb 5 m, roll step, yaw step), checks, plots
%   ok = rflymac_quicktest(false)  no plot
%   Requires `coptersim --ext` running. Same scenario and pass criteria as tests/test_client.c.

if nargin < 1, doPlot = true; end
F = rflymac_fields();
dt = 0.004; N = round(15 / dt);
y = rflymac_step('reset');
rflymac_attitude_ctrl(zeros(4,1), zeros(3,1), zeros(3,1), zeros(3,1), zeros(3,1), dt, true);
hist = zeros(numel(y), N);
for k = 1:N
    t = y(F.time);
    sp = [0.17 * (t >= 5 && t < 7); 0; 0.5 * (t >= 9); -5 * (t >= 0.5)];
    m = rflymac_attitude_ctrl(sp, y(F.euler), y(F.rate), y(F.pos), y(F.vel), dt, false);
    y = rflymac_step(m, round(dt / 0.001));
    hist(:, k) = y;
end
t = hist(F.time, :);
at = @(tt) find(abs(t - tt) < 1e-9, 1);
checks = {
    'lockstep: exactly 15 s',          abs(t(end) - 15) < 1e-9
    'altitude 5 m by t=4.9 s',         abs(hist(F.pos(3), at(4.9)) + 5) < 0.3
    'roll step tracked (~9.7 deg)',    abs(hist(F.euler(1), at(6.9)) - 0.17) < 0.03
    'yaw step tracked (~28.6 deg)',    abs(hist(F.euler(3), at(13)) - 0.5) < 0.05
    'altitude held at end',            abs(hist(F.pos(3), end) + 5) < 0.5 };
ok = true;
for i = 1:size(checks, 1)
    fprintf('%s: %s\n', ternary(checks{i, 2}, 'PASS', 'FAIL'), checks{i, 1});
    ok = ok && checks{i, 2};
end
if doPlot
    figure('Name', 'RflyMac quicktest');
    subplot(2, 1, 1); plot(t, -hist(F.pos(3), :)); grid on; ylabel('altitude (m)');
    title('coptersim + rflymac\_attitude\_ctrl');
    subplot(2, 1, 2); plot(t, rad2deg(hist(F.euler, :))); grid on;
    ylabel('attitude (deg)'); xlabel('t (s)'); legend('roll', 'pitch', 'yaw');
end
end

function s = ternary(c, a, b)
if c, s = a; else, s = b; end
end
