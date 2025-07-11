import numpy as np 

def calculate_jerk_kernel(S):
    '''
    here we calculate the 6x6 matrix H for the jerk cost integral. 
    J = integral from 0 to S of (l'"(s))^2ds, this equation can be found on the paper 
    under the M-step Spline QP path. main purpose of doing this is to translate one specific part of our desired path
    behavior which is smoothness into the mathematical language that the optimization solver understand whats going on.
    p0 is index 0 
    p1 is index 1 
    p2 is index 2
    p3 is index 3 
    p4 is index 4 
    p5 is index 5 

    FYI, parameters or values comes from the quintic polynomial derivatives.

    Main idea of this script is to converting the high-level integral formulas from the paper
    into the matrix format that the QP solver requires. 
    '''

    H = np.zeros((6,6))
    #integral of (6p3)^2 
    H[3,3] = 36 * S
    H[4,4] = 192 * S**3
    H[5,5] = 720 * S**5

    H[3,4] = H[4,3] = 144 * S**2
    H[3,5] = H[5,3] = 240 * S**3
    H[4,5] = H[5,4] = 720 * S**4

    return 2 * H # multiply by 2 to match the standard QP form


def calculate_accel_kernel(S):

    '''
    this one and vel kernel have the same idea with smoothness kernel, for accel kernel we use second derivatives
    its the same, just calculating the quintic polynomial where second derivatives are used. 
    '''

    H = np.zeros((6,6))

    H[2,2] = 4 * S
    H[3,3] = 12 * S**3
    H[4,4] = 144 * S**5 /5
    H[5,5] = 400 * S**7 /7
    
    H[2,3] = H[3,2] = 6 * S**2
    H[2,4] = H[4,2] = 16 * S**3
    H[2,5] = H[5,2] = 20 * S**4
    
    H[3,4] = H[4,3] = 36 * S**4
    H[3,5] = H[5,3] = 48 * S**5

    H[4,5] = H[5,4] = 80 * S**6

    return 2 * H


    

def calculate_vel_kernel(S):
    
    H = np.zeros((6, 6))

    H[1, 1] = S
    H[1, 2] = H[2, 1] = S**2
    H[1, 3] = H[3, 1] = S**3
    H[1, 4] = H[4, 1] = S**4
    H[1, 5] = H[5, 1] = S**5

    H[2, 2] = 4 * S**3 / 3
    H[2, 3] = H[3, 2] = 2 * S**4
    H[2, 4] = H[4, 2] = 2 * S**5
    H[2, 5] = H[5, 2] = 10 * S**6 / 3

    H[3, 3] = 9 * S**5 / 5
    H[3, 4] = H[4, 3] = 3 * S**6
    H[3, 5] = H[5, 3] = 3 * S**7
    H[4, 4] = 16 * S**7 / 7

    H[4, 5] = H[5, 4] = 4 * S**8
    H[5, 5] = 25 * S**9 / 9
    return 2 * H
    
def combine_kernels(H_jerk, H_accel, H_vel, w_jerk, w_accel, w_vel):
    '''
    combine all the matrices into a single weighed matrix for QP
    '''

    H_smooth = (w_jerk * H_jerk + w_accel * H_accel + w_vel * H_vel)

    return H_smooth


if __name__ == '__main__':
    S = 5.0
    
    np.set_printoptions(formatter={'float': '{:0.1f}'.format}, suppress=True)
    print("Jerk Kernel")
    H_jerk = calculate_jerk_kernel(S)
    print(H_jerk)

    print("\n Acceleration Kernel")
    H_accel = calculate_accel_kernel(S)
    print(H_accel)

    print("\n Velocity Kernel")
    H_vel = calculate_vel_kernel(S)
    print(H_vel)

    w_jerk = 1.0
    w_accel = 1.0
    w_vel = 1.0

    print("\n total overall smoothness kernel")
    H_smooth = combine_kernels(H_jerk, H_accel, H_vel, w_jerk, w_accel, w_vel)
    print(H_smooth)
