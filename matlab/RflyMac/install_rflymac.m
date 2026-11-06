function install_rflymac(varargin)
%INSTALL_RFLYMAC Build and install the RflyMac Simulink package (macOS).
%   install_rflymac            compile MEX files, add to path (this session), build models
%   install_rflymac('save')    ... and save the path for future MATLAB sessions
%
%   Needs: MATLAB + Simulink, and a C compiler for MEX (Xcode: run `mex -setup C` once).
%   No other toolboxes are required.

root   = fileparts(mfilename('fullpath'));
srcdir = fullfile(root, 'src');
bindir = fullfile(root, 'bin');
if ~exist(bindir, 'dir'), mkdir(bindir); end
if ~exist(fullfile(root, 'models'), 'dir'), mkdir(fullfile(root, 'models')); end

fprintf('RflyMac: installing from %s\n', root);

cc = mex.getCompilerConfigurations('C', 'Selected');
if isempty(cc)
    error('rflymac:nocompiler', ...
        'No C compiler selected for MEX. Install Xcode, then run:  mex -setup C');
end
fprintf('RflyMac: C compiler: %s\n', cc.Name);

inc = ['-I' srcdir];
client = fullfile(srcdir, 'rflymac_client.c');
mex('-silent', '-outdir', bindir, inc, fullfile(srcdir, 'sfun_rflymac.c'), client);
mex('-silent', '-outdir', bindir, '-output', 'rflymac_step', inc, ...
    fullfile(srcdir, 'rflymac_step_mex.c'), client);
fprintf('RflyMac: built sfun_rflymac and rflymac_step (%s)\n', mexext);

addpath(root, bindir, fullfile(root, 'models'));
if nargin >= 1 && strcmpi(varargin{1}, 'save')
    if savepath ~= 0
        warning('rflymac:savepath', 'Could not save the MATLAB path; run install_rflymac each session.');
    else
        fprintf('RflyMac: path saved.\n');
    end
end

if license('test', 'Simulink') && ~isempty(ver('simulink'))
    rflymac_build_library();
    rflymac_build_example();
else
    fprintf('RflyMac: Simulink not found; skipped the block library (rflymac_step still works).\n');
end

fprintf(['\nRflyMac ready.\n', ...
    '  1. In a terminal:   coptersim --ext            (lockstep server on UDP 20100)\n', ...
    '  2. In MATLAB:       rflymac_quicktest          (script test, no Simulink)\n', ...
    '                      open_system(''rflymac_example_attitude'')  then press Run\n']);
end
