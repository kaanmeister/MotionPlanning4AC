implemented and pushed collide checker, dp path planner, 5th order quintic polynomial calculations and kernel calculations for H matrix, there are still things to improve and finetune.

collide_checker.py will only visualize the paths which do not collide (shown in green) and do collide (which shown in red).

dp_path_planner.py will visualize a rough optimized single path which created by dynamic programming.

quintic_polynomial.py this script includes methods to calculate polynomial's value and its first, second, and third derivatives (heading, curvature, and jerk) at any point.

calc_kernels4matrixH.py this script translates the smoothness objectives from the paper into the matrix format rquired by QP solver. it provides functions to compute 6x6 kernel matrices for each component of the path's smoothness cost such as jerk,acceleration,and velocity

All of them will be used for the next following days to actually building the QP optimizer to create a new, smooth spline path.
