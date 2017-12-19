function set_matlab_utils_path()

% MAIN_PATH = '/home/user/as64_ws/as64_ws/matlab/utils/';
MAIN_PATH = '/home/slifer/Dropbox/64631466/lib/as64_ws/matlab/utils/';
% MAIN_PATH = 'C:/Users/Slifer/Dropbox/64631466/lib/as64_ws/matlab/utils/';
% MAIN_PATH = 'C:/Users/Slifer64/Dropbox/64631466/lib/as64_ws/matlab/utils/';


addpath(MAIN_PATH);

addpath_DMP_lib(MAIN_PATH);
% addpath_math_lib(MAIN_PATH);
addpath_filter_lib(MAIN_PATH);
% addpath_optimization_lib(MAIN_PATH);
% addpath_signalProcessing_lib(MAIN_PATH);
addpath_IO_lib(MAIN_PATH);
addpath_plot_lib(MAIN_PATH);

% addpath_GMR_lib(MAIN_PATH);
% addpath_SEDS_lib(MAIN_PATH);
% addpath_lwr4_lib(MAIN_PATH);
% addpath_cpp_code_gen_lib(MAIN_PATH);
% addpath_vrep_lib(MAIN_PATH);
% addpath_wrl_lib(MAIN_PATH);


DMP_TEST_PATH = '../DMP_utils/';
addpath([DMP_TEST_PATH 'DMP_data_process/']);
addpath([DMP_TEST_PATH 'DMP_plot_functions/']);
addpath([DMP_TEST_PATH 'DMP_train_data/']);


end
