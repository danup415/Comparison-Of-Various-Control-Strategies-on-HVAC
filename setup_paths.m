function setup_paths()
%SETUP_PATHS  Add project folders to the path. Run from the project root.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'config'));
addpath(genpath(fullfile(root, 'src')));
addpath(fullfile(root, 'simulink'));
addpath(fullfile(root, 'tests'));
addpath(fullfile(root, 'validation', 'energyplus'));
cd(root);
end
