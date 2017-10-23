function set_matlab_utils_path()

MATLAB_PATH = '/home/slifer/Dropbox/64631466/lib/matlab/utils/';
addpath([MATLAB_PATH 'SEDS_lib/']);
addpath([MATLAB_PATH 'GMR_lib/']);
addpath([MATLAB_PATH 'math_lib/']);
addpath([MATLAB_PATH 'math_lib/quaternion/']);
addpath([MATLAB_PATH 'lwr4_lib/']);
addpath([MATLAB_PATH 'plot_lib/']);
addpath([MATLAB_PATH 'wrl_lib/']);

DMP_PATH = '../DMP_utils/';
addpath([DMP_PATH 'DMP_data_process/']);
addpath([DMP_PATH 'DMP_plot_functions/']);
addpath([DMP_PATH 'DMP_train_data/']);


end
