function root = setup_paths()
%SETUP_PATHS  Add the folders of this release (src/, model/, model/generated/) to the MATLAB
%   path and return the release root folder. Every entry-point script calls it; paths are
%   resolved relative to this file, so the release can be placed anywhere.
root = fileparts(mfilename('fullpath'));
addpath(fullfile(root, 'src'), fullfile(root, 'model'), fullfile(root, 'model', 'generated'));
end
