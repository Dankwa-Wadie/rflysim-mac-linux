function f = rflymac_fields()
%RFLYMAC_FIELDS Index map for the 39x1 state vector returned by rflymac_step.
%   f = rflymac_fields();  y = rflymac_step(...);  z = y(f.pos(3));
f.time      = 1;
f.pos       = 2:4;     % NED (m)
f.vel       = 5:7;     % NED (m/s)
f.quat      = 8:11;    % w x y z
f.euler     = 12:14;   % roll pitch yaw (rad)
f.rate      = 15:17;   % p q r (rad/s)
f.accel     = 18:20;   % specific force, body (m/s^2)
f.gyro      = 21:23;   % rad/s
f.mag       = 24:26;   % gauss, body
f.baro_alt  = 27;      % m AMSL
f.gps       = 28:30;   % lat lon (deg), alt (m)
f.motor_w   = 31:38;   % rad/s
f.on_ground = 39;
end
