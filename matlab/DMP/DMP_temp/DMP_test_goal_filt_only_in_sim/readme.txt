Test DMP when training without goal filtering and use goal filtering only in simulation.

Also the graphs where no goal filtering is used neither in training nor in simulation for comparison.

Without goal filtering at all, everything looks fine.

With goal filtering during simulation, until time step 1.1 (that is until the goal 'g' reaches the final goal 'g0')
the output of the DMP deviates from the demonstration, especially at the beginning.

Also note that the shape attractor is the same, whether goal filtering is used or not.

This is to be expected, since the goal filtering is not taken into account during DMP training.
Therefore
ddy = f_g + f_s
where f_g denotes the goal attractor and f_s the shape attractor
is different when goal filtering is used since f_g changes while f_s remains the same as if no
goal filtering is used. This produces an erroneous ddy thus the DMP movement deviation from the desired one.

