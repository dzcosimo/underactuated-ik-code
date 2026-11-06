function setup_paths()
%SETUP_PATHS  Add src/, model/ and model/generated/ of the double_pendulum folder to the path.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'), fullfile(root, 'model'), fullfile(root, 'model', 'generated'));
end
