function set_matlab_utils_path()

MATLAB_PATH = '/home/leonidas/Documents/workspaces/ros_ws/as64_ws0/matlab/utils/';
%MATLAB_PATH = 'C:/Users/Slifer/Dropbox/64631466/lib/as64_ws/matlab/utils/';

addpath([MATLAB_PATH 'math_lib/']);
addpath([MATLAB_PATH 'filter_lib/']);
addpath([MATLAB_PATH 'IO_lib/']);
addpath([MATLAB_PATH 'math_lib/quaternion/']);
addpath([MATLAB_PATH 'lwr4_lib/']);
addpath([MATLAB_PATH 'plot_lib/']);
addpath([MATLAB_PATH 'wrl_lib/']);

addpath([MATLAB_PATH 'SEDS_lib/']);
addpath([MATLAB_PATH 'GMR_lib/']);

addpath([MATLAB_PATH 'DMP_lib/DMP']);
addpath([MATLAB_PATH 'DMP_lib/utils']);
addpath([MATLAB_PATH 'DMP_lib/CanonicalSystem']);


DMP_PATH = '../DMP_utils/';
addpath([DMP_PATH 'DMP_data_process/']);
addpath([DMP_PATH 'DMP_plot_functions/']);
addpath([DMP_PATH 'DMP_train_data/']);


end
