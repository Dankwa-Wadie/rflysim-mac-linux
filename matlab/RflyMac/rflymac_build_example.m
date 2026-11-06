function rflymac_build_example()
%RFLYMAC_BUILD_EXAMPLE Create models/rflymac_example_attitude.slx.
%   Simulink harness like RflySim's px4demo_attitude_plant, but the plant is coptersim:
%   setpoint steps -> Controller (MATLAB Function: rflymac_attitude_ctrl) -> CopterSim (Mac)
%   -> feedback, scopes and To Workspace logging.

root = fileparts(mfilename('fullpath'));
outdir = fullfile(root, 'models');
mdl = 'rflymac_example_attitude';
if bdIsLoaded(mdl), close_system(mdl, 0); end
f = fullfile(outdir, [mdl '.slx']);
if exist(f, 'file'), delete(f); end
load_system('rflymac_lib');

new_system(mdl);
set_param(mdl, 'Solver', 'FixedStepDiscrete', 'FixedStep', '0.004', 'StopTime', '15');

% --- Setpoints: [roll; pitch; yaw; z] ---
add_block('simulink/Sources/Step', [mdl '/roll 10deg @5s'], 'Time', '5', 'Before', '0', ...
    'After', '0.17', 'Position', [30 30 60 60]);
add_block('simulink/Sources/Step', [mdl '/roll back @7s'], 'Time', '7', 'Before', '0', ...
    'After', '-0.17', 'Position', [30 80 60 110]);
add_block('simulink/Math Operations/Sum', [mdl '/roll sp'], 'Inputs', '++', ...
    'Position', [90 50 110 90]);
add_block('simulink/Sources/Constant', [mdl '/pitch sp'], 'Value', '0', 'Position', [30 130 60 160]);
add_block('simulink/Sources/Step', [mdl '/yaw 0.5rad @9s'], 'Time', '9', 'Before', '0', ...
    'After', '0.5', 'Position', [30 180 60 210]);
add_block('simulink/Sources/Step', [mdl '/climb to 5m @0.5s'], 'Time', '0.5', 'Before', '0', ...
    'After', '-5', 'Position', [30 230 60 260]);
add_block('simulink/Signal Routing/Mux', [mdl '/sp'], 'Inputs', '4', 'Position', [150 40 155 270]);
add_line(mdl, 'roll 10deg @5s/1', 'roll sp/1');
add_line(mdl, 'roll back @7s/1', 'roll sp/2');
add_line(mdl, 'roll sp/1', 'sp/1');
add_line(mdl, 'pitch sp/1', 'sp/2');
add_line(mdl, 'yaw 0.5rad @9s/1', 'sp/3');
add_line(mdl, 'climb to 5m @0.5s/1', 'sp/4');

% --- Controller (MATLAB Function block calling rflymac_attitude_ctrl.m) ---
ctrl = [mdl '/Controller'];
add_block('simulink/User-Defined Functions/MATLAB Function', ctrl, 'Position', [220 100 360 260]);
ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', ctrl);
ch.Script = sprintf(['function motors = Controller(sp, euler, rate, pos, vel)\n', ...
    '%% Replace this call with your own controller.\n', ...
    'motors = rflymac_attitude_ctrl(sp, euler, rate, pos, vel, 0.004, false);\n']);

% --- Plant: coptersim ---
sim = [mdl '/CopterSim (Mac)'];
add_block('rflymac_lib/CopterSim (Mac)', sim, 'Position', [430 60 570 400]);
add_line(mdl, 'sp/1', 'Controller/1');
add_line(mdl, 'Controller/1', 'CopterSim (Mac)/1');
% outputs: 1 time 2 pos 3 vel 4 quat 5 euler 6 rate 7 accel 8 gyro 9 mag 10 baro 11 gps 12 motor_w 13 on_ground
add_line(mdl, 'CopterSim (Mac)/5', 'Controller/2', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/6', 'Controller/3', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/2', 'Controller/4', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/3', 'Controller/5', 'autorouting', 'on');

% --- Scopes and logging ---
add_block('simulink/Sinks/Scope', [mdl '/Attitude (rad)'], 'Position', [700 120 730 150]);
add_block('simulink/Sinks/Scope', [mdl '/Position NED (m)'], 'Position', [700 40 730 70]);
add_block('simulink/Sinks/To Workspace', [mdl '/log euler'], 'VariableName', 'rfly_euler', ...
    'SaveFormat', 'Structure With Time', 'Position', [700 170 760 190]);
add_block('simulink/Sinks/To Workspace', [mdl '/log pos'], 'VariableName', 'rfly_pos', ...
    'SaveFormat', 'Structure With Time', 'Position', [700 210 760 230]);
add_line(mdl, 'CopterSim (Mac)/2', 'Position NED (m)/1', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/5', 'Attitude (rad)/1', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/5', 'log euler/1', 'autorouting', 'on');
add_line(mdl, 'CopterSim (Mac)/2', 'log pos/1', 'autorouting', 'on');
for p = [1 4 7 8 9 10 11 12 13]
    t = sprintf('%s/term%d', mdl, p);
    add_block('simulink/Sinks/Terminator', t, 'Position', [620 20+30*p 640 35+30*p]);
    add_line(mdl, sprintf('CopterSim (Mac)/%d', p), sprintf('term%d/1', p), 'autorouting', 'on');
end

save_system(mdl, f);
close_system(mdl);
fprintf('RflyMac: wrote %s\n', f);
end
