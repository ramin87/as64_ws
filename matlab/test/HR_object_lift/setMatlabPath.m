function setMatlabPath()

    MAIN_PATH = [pwd '/../../utils/'];
    
    addpath([pwd '/utils/']);
    
    addpath(MAIN_PATH);
    addpath_DMP_lib(MAIN_PATH);
    addpath_filter_lib(MAIN_PATH);
    addpath_io_lib(MAIN_PATH);
    addpath_plot_lib(MAIN_PATH);

end