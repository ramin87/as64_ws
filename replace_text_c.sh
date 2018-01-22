#!/bin/bash
# A sample Bash script, by Ryan

## declare initial text
declare -a init_text=("IO-lib"  
					  )

## declare new text
declare -a new_text=("io-lib"
					 )

# get length of an array
arraylength=${#init_text[@]}

# use for loop to read all values and indexes
for (( i=0; i<${arraylength}; i++ ));
do
  grep -rl ${init_text[$i]} ./c++/ | xargs sed -i 's/'${init_text[$i]}'/'${new_text[$i]}'/g'
done






