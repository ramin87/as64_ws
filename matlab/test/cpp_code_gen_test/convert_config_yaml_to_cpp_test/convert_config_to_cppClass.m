clc;
close all;
clear;

%% include path
% MATLAB_PATH = 'C:/Users/Slifer/Dropbox/64631466/lib/as64_ws/matlab/utils/';
MAIN_PATH = '../../../utils/';
addpath_cpp_code_gen_lib(MAIN_PATH);

%% arguments
config_filename = 'config.yml';
out_filename = 'cmd_args';

%% convert yaml file to '<out_filename>.h' and '<out_filename>.cpp' file
convert_config_yaml_to_cpp(config_filename, out_filename)
