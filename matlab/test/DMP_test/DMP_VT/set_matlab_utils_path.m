function set_matlab_utils_path()

MAIN_PATH = [pwd '/../../../utils/'];


addpath(MAIN_PATH);

addpath_DMP_lib(MAIN_PATH);
% addpath_math_lib(MAIN_PATH);
addpath_filter_lib(MAIN_PATH);
% addpath_optimization_lib(MAIN_PATH);
% addpath_signalProcessing_lib(MAIN_PATH);
addpath_io_lib(MAIN_PATH);
addpath_plot_lib(MAIN_PATH);

% addpath_GMR_lib(MAIN_PATH);
% addpath_SEDS_lib(MAIN_PATH);
% addpath_lwr4_lib(MAIN_PATH);
% addpath_cpp_code_gen_lib(MAIN_PATH);
% addpath_vrep_lib(MAIN_PATH);
% addpath_wrl_lib(MAIN_PATH);


DMP_TEST_PATH = '../DMP_utils/';
addpath([DMP_TEST_PATH 'DMP_plot_functions/']);


end
