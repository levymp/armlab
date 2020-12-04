"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm



def clamp(angle):
    """!
    @brief      Clamp angles between (-pi, pi]

    @param      angle  The angle

    @return     Clamped angle
    """
    while angle > np.pi:
        angle -= 2 * np.pi
    while angle <= -np.pi:
        angle += 2 * np.pi
    return angle

clamp_mtx = np.vectorize(clamp)

def FK_dh(dh_params, joint_angles, link):
    """!
    @brief      Get the 4x4 transformation matrix from link to world

                TODO: implement this function

                Calculate forward kinematics for rexarm using DH convention

                return a transformation matrix representing the pose of the desired link

                note: phi is the euler angle about the y-axis in the base frame

    @param      dh_params     The dh parameters as a 2D list each row represents a link and has the format [a, alpha, d,
                              theta]
    @param      joint_angles  The joint angles of the links
    @param      link          The link to transform from...

    @return     a transformation matrix representing the pose of the desired link
    """
    H_mat = np.eye(4)

    for l in range(link):
        H_mat = H_mat*get_transform_from_dh(dh_params[l, 0], dh_params[l, 1], dh_params[l, 2], joint_angles[l] + dh_params[l,3])

    return H_mat


def get_transform_from_dh(a, alpha, d, theta):
    """!
    @brief      Gets the transformation matrix from dh parameters.

    TODO: Find the T matrix from a row of a DH table

    @param      a      a meters
    @param      alpha  alpha radians
    @param      d      d meters
    @param      theta  theta radians

    @return     The 4x4 transform matrix.
    """
    st = np.sin(theta)
    ct = np.cos(theta)
    sa = np.sin(alpha)
    ca = np.cos(alpha)

    transform_matrix = np.matrix([[ct, -st*ca, st*sa, a*ct],
                                 [st, ct*ca, -ct*sa, a*st],
                                 [0, sa, ca, d],
                                 [0, 0, 0, 1]])

    return transform_matrix


def get_euler_angles_from_T(T):
    """!
    @brief      Gets the euler angles from a transformation matrix.

                TODO: Implement this function return the Euler angles from a T matrix

    @param      T     transformation matrix


    @return     The euler angles from T.
    """
    
    # phi, theta, psi
    return [np.arctan2(T[2,1],T[2,2]), np.arcsin(T[2,0]), np.arctan2(T[1, 0],T[0, 0])]


def get_pose_from_T(T):
    """!
    @brief      Gets the pose from T.

                TODO: implement this function return the joint pose from a T matrix of the form (x,y,z,phi) where phi is
                rotation about base frame y-axis

    @param      T     transformation matrix

    @return     The pose from T.
    """
    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]
    phi = np.arctan2(T[2,1],T[2,2])

    return (x, y, z, phi)


def FK_pox(joint_angles, m_mat, s_lst):
    """!
    @brief      Get a 4-tuple (x, y, z, phi) representing the pose of the desired link

                TODO: implement this function, Calculate forward kinematics for rexarm using product of exponential
                formulation return a 4-tuple (x, y, z, phi) representing the pose of the desired link note: phi is the euler
                angle about y in the base frame

    @param      joint_angles  The joint angles
                m_mat         The M matrix
                s_lst         List of screw vectors

    @return     a 4-tuple (x, y, z, phi) representing the pose of the desired link
    """
    pass


def to_s_matrix(w,v):
    """!
    @brief      Convert to s matrix.

    TODO: implement this function
    Find the [s] matrix for the POX method e^([s]*theta)

    @param      w     { parameter_description }
    @param      v     { parameter_description }

    @return     { description_of_the_return_value }
    """
    pass


def IK_geometric(dh_params, pose):
    """!
    @brief      Get all possible joint configs that produce the pose.

                TODO: Convert a desired end-effector pose as np.array x,y,z,phi to joint angles

    @param      dh_params  The dh parameters
    @param      pose       The desired pose as np.array x,y,z,phi

    @return     All four possible joint configurations in a numpy array 4x4 where each row is one possible joint
                configuration
    """
    x = pose[0]
    y = pose[1]
    z = pose[2]
    phi = pose[3]
    
    l2 = dh_params[1,0] # link 2 length
    l3 = dh_params[2,0] # link 3 length
    t2 = dh_params[1,3]
    t3 = dh_params[2,3]

    # Get base rotation
    if(abs(x) < 0.005 and abs(y) < 0.005):
        theta1_s1 = 0
        theta2_s2 = 0
    else:
        theta1_s1 = clamp(np.arctan2(y,x)) # base rotation
        theta1_s2 = clamp(np.pi + theta1_s1) # second solution to theta 1

    end_link = dh_params[3,0]
    k = np.linalg.norm([x,y]) # length along x-y
    ok = k - end_link*np.cos(phi)
    oz = z - end_link*np.sin(phi)

    num = ok**2 + oz**2 - l2**2 - l3**2
    den = 2*l2*l3

    theta3_s1 = np.arccos(num/den)
    theta3_s1 = np.sign(theta3_s1)*theta3_s1 # make this always positive
    theta3_s2 = -theta3_s1
    print(num, den)
    print(num/den)
    if(abs(num/den) < 0.03):
        theta3_s1 = -np.pi/2
        theta3_s2 = np.pi/2
        print('called t3 edit')
    elif(abs(num/den) > 0.95):
        theta3_s1 = -np.pi
        theta3_s2 = np/pi

    alpha_s1 = np.arctan2(l3*np.sin(theta3_s1), l2 + l3*np.cos(theta3_s1))
    alpha_s2 = np.arctan2(l3*np.sin(theta3_s2), l2 + l3*np.cos(theta3_s2))

    theta2_s1 = np.arctan2(oz,ok) - alpha_s1
    theta2_s2 = np.arctan2(oz,ok) - alpha_s2

    theta4_s1 = phi - sum([theta2_s1, theta3_s1])
    theta4_s2 = phi - sum([theta2_s2, theta3_s2])
    theta4_s3 = phi - sum([theta2_s1, theta3_s1])
    theta4_s4 = phi - sum([theta2_s2, theta3_s2])

    theta2_s1 -= t2
    theta2_s2 -= t2
    theta3_s1 -= t3
    theta3_s2 -= t3

    if(abs(theta1_s1) > abs(theta1_s2)):
        tmp = theta1_s2
        theta1_s2 = theta1_s1
        theta1_s1 = tmp
    solution_matrix = np.matrix([[theta1_s1, theta2_s1, theta3_s1, theta4_s1],
                                 [theta1_s1, theta2_s2, theta3_s2, theta4_s2],
                                 [theta1_s2, -theta2_s1, -theta3_s1, -theta4_s3],
                                 [theta1_s2, -theta2_s2, -theta3_s2, -theta4_s4]])

    solution_matrix = clamp_mtx(solution_matrix)

    return solution_matrix