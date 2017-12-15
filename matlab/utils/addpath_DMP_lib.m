function addpath_DMP_lib(MAIN_PATH)

	addpath([MAIN_PATH '/DMP_lib/DMP/']);
    addpath([MAIN_PATH '/DMP_lib/utils/']);
    addpath([MAIN_PATH '/DMP_lib/CanonicalSystem/']);
    addpath([MAIN_PATH '/DMP_lib/CanonicalSystem/CanonicalClock/']);
    addpath([MAIN_PATH '/DMP_lib/CanonicalSystem/CanonicalFunction/']);
    
    % add dependencies from other libraries
    addpath_math_lib(MAIN_PATH);
    addpath_optimization_lib(MAIN_PATH);
    addpath_signalProcessing_lib(MAIN_PATH);

end
