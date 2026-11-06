function rflymac_build_library()
%RFLYMAC_BUILD_LIBRARY Create models/rflymac_lib.slx containing the "CopterSim (Mac)" block.

root = fileparts(mfilename('fullpath'));
outdir = fullfile(root, 'models');
if ~exist(outdir, 'dir'), mkdir(outdir); end
lib = 'rflymac_lib';
if bdIsLoaded(lib), close_system(lib, 0); end
f = fullfile(outdir, [lib '.slx']);
if exist(f, 'file'), delete(f); end

new_system(lib, 'Library');
blk = [lib '/CopterSim (Mac)'];
add_block('simulink/User-Defined Functions/S-Function', blk, ...
    'FunctionName', 'sfun_rflymac', 'Parameters', 'port, Ts, host', ...
    'Position', [100 60 260 380]);

m = Simulink.Mask.create(blk);
m.Type = 'CopterSim (Mac)';
m.Description = sprintf(['Lockstep link to the rflysim_macV1 simulator (coptersim --ext).\n', ...
    'Input: motor throttles 0..1 (PX4 order, up to 8).\n', ...
    'Start `coptersim --ext` before running the model.']);
m.addParameter('Name', 'port', 'Prompt', 'UDP port', 'Value', '20100', 'Type', 'edit');
m.addParameter('Name', 'Ts', 'Prompt', 'Sample time Ts (s, multiple of 0.001)', ...
    'Value', '0.004', 'Type', 'edit');
m.addParameter('Name', 'host', 'Prompt', 'Host', 'Value', '127.0.0.1', 'Type', 'edit', ...
    'Evaluate', 'off');
labels = {'time', 'pos NED', 'vel NED', 'quat', 'euler', 'rates pqr', 'accel', 'gyro', ...
    'mag', 'baro alt', 'gps', 'motor w', 'on ground'};
dispstr = sprintf('disp(''CopterSim\\n(Mac)'');\nport_label(''input'',1,''motors'');\n');
for i = 1:numel(labels)
    dispstr = [dispstr sprintf('port_label(''output'',%d,''%s'');\n', i, labels{i})]; %#ok<AGROW>
end
m.Display = dispstr;

set_param(lib, 'EnableLBRepository', 'on');   % show in the Library Browser
save_system(lib, f);
close_system(lib);
fprintf('RflyMac: wrote %s\n', f);
end
