#!/bin/bash
# A sample Bash script, by Ryan

## declare initial text
declare -a init_text=("kernel_function" 
					  "get_states_dot" 
					  "forcing_term"
					  "gaussian_kernel"
					  "_ptr"
					  "kernel_std_scaling"
					  "set_training_params"
					  "train_method"
					  "calc_Fd"
					  "learned_forcing_term"
					  "goal_attractor"
					  "shape_attractor"
					  "shape_attr_gating"
					  "goal_attr_gating"
					  "get_tau"
					  "set_tau"
					  "get_phase_dot"
					  "set_gating_fun_params"
					  "get_output"
					  "get_output_dot"
					  "qPos2quat"
					  "quat2qPos"
					  "forcingTerm_scaling"
					  "temporalAlignSignals"
					  "spatialAlignSignals"
					  "leastSquares"
					  "getOutput_dot"
					  "normalizedKernelSumLeastSquares"
					  )

## declare new text
declare -a new_text=("kernelFunction" 
                     "getStatesDot" 
                     "forcingTerm"
                     "gaussianKernel"
                     "Ptr"
                     "kernelStdScaling"
                     "setTrainingParams"
                     "trainMethod"
                     "calcFd"
                     "learnedForcingTerm"
                     "goalAttractor"
                     "shapeAttractor"
                     "shapeAttrGating"
                     "goalAttrGating"
                     "getTau"
					 "setTau"
					 "getPhaseDot"
					 "setGatingFunParams"
					 "getOutput"
					 "getOutputDot"
					 "qpos2quat"
					 "quat2qpos"
					 "forcingTermScaling"
					 "makeSignalsEqualLength"
					 "alignSignalsWithDTW"
					 "normalizedKernelSumLeastSquares"
					 "getOutputDot"
					 "normKernelLS"
					 )

# get length of an array
arraylength=${#init_text[@]}

# use for loop to read all values and indexes
for (( i=0; i<${arraylength}; i++ ));
do
  grep -rl ${init_text[$i]} ./matlab/ | xargs sed -i 's/'${init_text[$i]}'/'${new_text[$i]}'/g'
done






