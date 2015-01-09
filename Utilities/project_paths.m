function folders = project_paths()
%project_paths   Define the set of folders to add to the MATLAB path
%  
%   Definition of the folders to add to the MATLAB path when this Project
%   is opened, and remove from the path when it is closed. Edit the
%   definition of folders below to add your own paths to the current
%   project.
%
%   The variable folders is a cell-array of paths relative to the project
%   root. For example,
%
%       folders = { ...
%           'data', ...
%           'models', ...
%           'src', ...
%           fullfile('components','core'), ...
%           'utilities' ...
%           };
%
%   Using the MATLAB command fullfile when coQnstructing folder hierarchies
%   will make your project compatible with multiple operating systems
%   (for example, both Windows and Linux).

%   Copyright 2011-2012 The MathWorks, Inc.

folders = { ...
    'Utilities', ...
    'Functions',...
    'DP_storage' ...
    
    };
