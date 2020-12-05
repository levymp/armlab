"""!
Implements Forward and Inverse kinematics with DH parametrs and product of exponentials

TODO: Here is where you will write all of your kinematics functions
There are some functions to start with, you may need to implement a few more
"""

import numpy as np
# expm is a matrix exponential function
from scipy.linalg import expm
from scipy.spatial.transform import Rotation as R



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
    # print(T)
    # print(T[0:2,0:2])
    r = R.from_dcm(T[0:3,0:3])

    # phi, theta, psi
    return r.as_euler('xyz', degrees=False)


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
    phi = get_euler_angles_from_T(T)[1]

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

def IK_2DOF(ok,oz,l2,l3):
    num = np.square(ok, dtype=np.float64) + np.square(oz, dtype=np.float64) - np.square(l2, dtype=np.float64) - np.square(l3, dtype=np.float64)
    den = 2*l2*l3

    res = np.divide(num,den, dtype=np.float64)
    
    theta3_s1 = np.arctan2(np.sqrt(1.0-np.square(res, dtype=np.float64), dtype=np.float64), res, dtype=np.float64)
    theta3_s1 = np.sign(theta3_s1)*theta3_s1 # make this always positive
    theta3_s2 = -theta3_s1

    alpha_s1 = np.arctan2(l3*np.sin(theta3_s1, dtype=np.float64), l2 + l3*np.cos(theta3_s1, dtype=np.float64), dtype=np.float64)
    alpha_s2 = np.arctan2(l3*np.sin(theta3_s2, dtype=np.float64), l2 + l3*np.cos(theta3_s2, dtype=np.float64), dtype=np.float64)

    theta2_s1 = np.arctan2(oz,ok, dtype=np.float64) - alpha_s1
    theta2_s2 = np.arctan2(oz,ok, dtype=np.float64) - alpha_s2
    return theta2_s1,theta3_s1,theta2_s2,theta3_s2

def IK_geometric(dh_params, pose, theta5=0):
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
    
    l1 = dh_params[0,0] # link 1 length
    l2 = dh_params[1,0] # link 2 length
    l3 = dh_params[2,0] # link 3 length
    l4 = dh_params[3,0] # end link length

    if(np.sqrt(x**2 + y**2 + z**2) > (l1+l2+l3+l4)):
        print("NO SOLUTION FOUND")
        return False, np.zeros((4,5))
    t2 = dh_params[1,3]
    t3 = dh_params[2,3]

    theta1_s1 = clamp(np.arctan2(y,x)) # base rotation
    theta1_s2 = clamp(np.pi + theta1_s1) # second solution to theta 1
    
    # By convention, we will define the first solution as smallest angle
    swapped = False
    if(abs(theta1_s1) > abs(theta1_s2)):
        tmp = theta1_s2
        theta1_s2 = theta1_s1
        theta1_s1 = tmp

    k = np.sign(x)*np.sqrt((x**2 + y**2)) # length along x-y
    ok = k - l4*np.cos(phi)
    ok2 = -ok
    oz = z - l4*np.sin(phi)
    
    theta2_s1, theta3_s1, theta2_s2, theta3_s2 = IK_2DOF(ok,oz,l2,l3)

    theta2_s3 = np.pi - theta2_s1
    theta2_s4 = np.pi - theta2_s2

    theta3_s3 = -theta3_s1
    theta3_s4 = -theta3_s2

    theta4_s1 = phi - clamp(sum([theta2_s1, theta3_s1]))
    theta4_s2 = phi - clamp(sum([theta2_s2, theta3_s2]))
    theta4_s3 = phi - clamp(theta2_s3 + theta3_s3)
    theta4_s4 = phi - clamp(theta2_s4 + theta3_s4)

    theta2_s1 -= t2
    theta2_s2 -= t2
    theta2_s3 -= t2
    theta2_s4 -= t2

    theta3_s1 -= t3
    theta3_s2 -= t3
    theta3_s3 -= t3
    theta3_s4 -= t3

    solution_matrix = np.matrix([[theta1_s1, theta2_s1, theta3_s1, theta4_s1, theta5],
                                 [theta1_s1, theta2_s2, theta3_s2, theta4_s2, theta5],
                                 [theta1_s2, theta2_s3, theta3_s3, theta4_s3, theta5],
                                 [theta1_s2, theta2_s4, theta3_s4, theta4_s4, theta5]])

    solution_matrix = clamp_mtx(solution_matrix)

    # check if all results are nan
    if np.isnan(theta4_s1) and np.isnan(theta4_s2) and np.isnan(theta4_s3) and np.isnan(theta4_s4):
        print("NO SOLUTION FOUND")
        return False, solution_matrix

    # fk_poses = []
    # for joint_angles in solution_matrix.tolist():
    #     print('Joint angles:', joint_angles)
    #     for i, _ in enumerate(joint_angles):
    #         pose = get_pose_from_T(FK_dh(dh_params, joint_angles, i))
    #         if i == len(joint_angles)-1:
    #             print('Link {} pose: {}'.format(i, pose))
    #             fk_poses.append(pose)
    #     print()
    return True, solution_matrix