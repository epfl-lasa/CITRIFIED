%% get path for tutorial
path = which('setup_toolbox');
path = fileparts(path);

%% remove any auxiliary folder from the search path
restoredefaultpath();

%% remove the default user-specific path
userpath('clear');

%% add only the tp + ML_toolbox path
if exist('./ML_toolbox-master', 'dir')
    addpath(genpath([path '/ML_toolbox-master']));
elseif exist('./ML_toolbox', 'dir')
    addpath(genpath([path '/ML_toolbox']));
end
