function motors = rflymac_attitude_ctrl(sp, euler, rate, pos, vel, dt, reset)
%RFLYMAC_ATTITUDE_CTRL Example quadrotor controller (altitude + attitude), PX4 quad-x mixing.
%   motors = rflymac_attitude_ctrl(sp, euler, rate, pos, vel, dt, reset)
%     sp    = [roll; pitch; yaw; z]   setpoints (rad, rad, rad, m NED: -5 = 5 m up)
%     euler = [roll; pitch; yaw]      rate = [p; q; r]   pos/vel = NED
%     dt    = controller period (s)   reset = true to clear integrators
%     motors: 8x1 throttles 0..1 (1 FR, 2 RL, 3 FL, 4 RR; 5-8 unused)
%
%   Structure mirrors RflySim's px4demo_attitude_control (angle P -> rate PI per axis) plus an
%   altitude position->velocity cascade. Code-generation compatible (MATLAB Function block).
%   Kept in sync with the C reference in tests/test_client.c (verified against coptersim).

%#codegen
persistent iz ir
if isempty(iz) || reset
    iz = 0;
    ir = zeros(3, 1);
end

HOVER   = 0.608;            % hover throttle of the default RflySim quad (ModelData.db)
KP_Z    = 1.0;  VZ_MAX = 2.0;
KP_VZ   = 0.05; KI_VZ  = 0.02;
KP_ANG  = [5.0; 5.0; 3.0];
KP_RATE = [0.03; 0.03; 0.2];
KI_RATE = [0.02; 0.02; 0.05];

sp = sp(:); euler = euler(:); rate = rate(:); pos = pos(:); vel = vel(:);

% Altitude: position -> climb-rate setpoint -> throttle (NED: negative z = up)
ez    = sp(4) - pos(3);
vz_sp = clamp(KP_Z * ez, -VZ_MAX, VZ_MAX);
iz    = clamp(iz + KI_VZ * (vel(3) - vz_sp) * dt, -0.1, 0.1);
thr   = HOVER + KP_VZ * (vel(3) - vz_sp) + iz;
thr   = thr / max(cos(euler(1)) * cos(euler(2)), 0.5);
thr   = clamp(thr, 0.05, 0.95);

% Attitude: angle P -> rate PI
tau = zeros(3, 1);
for i = 1:3
    e = sp(i) - euler(i);
    if i == 3
        e = atan2(sin(e), cos(e));      % wrap yaw error
    end
    er    = KP_ANG(i) * e - rate(i);
    ir(i) = clamp(ir(i) + KI_RATE(i) * er * dt, -0.1, 0.1);
    tau(i) = KP_RATE(i) * er + ir(i);
end

% PX4 quad-x mixer: 1 FR(CCW) 2 RL(CCW) 3 FL(CW) 4 RR(CW)
motors = zeros(8, 1);
motors(1) = thr - tau(1) + tau(2) + tau(3);
motors(2) = thr + tau(1) - tau(2) + tau(3);
motors(3) = thr + tau(1) + tau(2) - tau(3);
motors(4) = thr - tau(1) - tau(2) - tau(3);
motors(1:4) = min(max(motors(1:4), 0), 1);
end

function y = clamp(x, lo, hi)
y = min(max(x, lo), hi);
end
