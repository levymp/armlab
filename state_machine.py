"""!
The state machine that implements the logic.
"""
from kinematics import IK_geometric
from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
import time
import numpy as np
import rospy
import subprocess
import os
from dance import dance_waypoints, dance_gripper



class StateMachine():
    """!
    @brief      This class describes a state machine.

                TODO: Add states and state functions to this class to implement all of the required logic for the armlab
    """

    def __init__(self, rxarm, camera):
        """!
        @brief      Constructs a new instance.

        @param      rxarm   The rxarm
        @param      planner  The planner
        @param      camera   The camera
        """
        self.rxarm = rxarm
        self.camera = camera
        self.status_message = "State: Idle"
        self.current_state = "idle"
        self.next_state = "idle"
        self.waypoints = []

        self.gripper_waypoints = []


    def set_next_state(self, state):
        """!
        @brief      Sets the next state.

            This is in a different thread than run so we do nothing here and let run handle it on the next iteration.

        @param      state  a string representing the next state.
        """
        self.next_state = state

    def run(self):
        """!
        @brief      Run the logic for the next state

                    This is run in its own thread.

                    TODO: Add states and funcitons as needed.
        """
        if self.next_state == "initialize_rxarm":
            self.initialize_rxarm()

        if self.next_state == "idle":
            self.idle()

        if self.next_state == "estop":
            self.estop()

        if self.next_state == "execute":
            self.execute()

        if self.next_state == "calibrate":
            self.calibrate()

        if self.next_state == "detect":
            self.detect()

        if self.next_state == "manual":
            self.manual()

        if self.next_state == "append_state_open":
            self.save_state(True)
            
        if self.next_state == "append_state_closed":
            self.save_state(False)

        if self.next_state == "pick":
            self.pick_idle()

        if self.next_state == "place":
            self.place_idle()


    """Functions run for each state"""

    def pick(self, point):
        print("angle", self.camera.block_contours)
        # go above the block
        found = False
        i = 0
        self.phi = -np.pi/2
        while (not found) and (i<=5):
            goal_pose1 = [point[0], point[1], point[2] + .1, self.phi]
            dx =  self.camera.block_contours[0][2][0] - self.camera.block_contours[0][1][0]
            dy =  self.camera.block_contours[0][2][1] - self.camera.block_contours[0][1][1]

            t5 = np.arctan2(dy,dx) + np.arctan2(point[1],point[0])
            print("t5", t5)
            success1, joint_angles1 = IK_geometric(self.rxarm.dh_params, goal_pose1, t5[0]) 
            joint_angles1 = np.asarray(joint_angles1)[1,:].tolist()
            # list(np.array(joint_angles1).reshape(-1,))

            # grab block
            goal_pose2 = [point[0], point[1], point[2], self.phi]
            success2, joint_angles2 = IK_geometric(self.rxarm.dh_params, goal_pose2, t5[0])
            joint_angles2 = np.asarray(joint_angles2)[1,:].tolist()
            if not (success1 and success2):
                print("ERROR: NO SOLUTION FOUND FOR IK")
            else:
                found = True
            i += 1
            self.phi += .25

        
        # list(np.array(joint_angles2).reshape(-1,))

        if not (success1 and success2):
            print("ERROR: NO SOLUTION FOUND FOR IK")
            return
        # print out first Joint Waypoints
        print('JOINT ANGLES (PICK):')
        print(np.array(joint_angles1) * 180/np.pi)
        self.waypoints = [joint_angles1, joint_angles1, joint_angles2, joint_angles1]
        self.gripper_waypoints = [1, 1, 0, 0]
        self.next_state = 'execute'

    def place(self, point):
        
        # go above the desired location
        goal_pose1 = [point[0], point[1], point[2] + .1, self.phi]
        success1, joint_angles1 = IK_geometric(self.rxarm.dh_params, goal_pose1)
        joint_angles1 = np.asarray(joint_angles1)[1,:].tolist()
        
        # release
        goal_pose2 = [point[0], point[1], point[2] + .05, self.phi]
        success2, joint_angles2 = IK_geometric(self.rxarm.dh_params, goal_pose2)
        joint_angles2 = np.asarray(joint_angles2)[1,:].tolist()

        if not (success1 and success2):
            print("ERROR: NO SOLUTION FOUND FOR IK")
            return

        self.waypoints = [joint_angles1, joint_angles1, joint_angles2, joint_angles1]
        self.gripper_waypoints = [0, 0, 1, 1]
        print(self.waypoints)
        self.next_state = 'execute'

    def pick_idle(self):
        self.status_message = "State: Pick - Waiting for input"
        self.next_state = "pick"

    def place_idle(self):
        self.status_message = "State: Place - Waiting for input"
        self.next_state = "place"
        
    def save_state(self, gripper_open):
        # get joint state
        tmp = self.rxarm.get_positions()
        print("NEW STATE: " + str(tmp.tolist()))

        # status message
        self.status_message = 'APPENDING NEW STATE'
        
        # save state
        self.waypoints.append(tmp.tolist())
        # save gripper state
        self.gripper_waypoints.append(gripper_open)
        
        # set new state
        self.next_state = "idle"


    def reset_waypoints(self):
        print('RESETING WAYPOINTS!')
        self.status_message = 'RESETING WAYPOINTS!'
        # Rest joint waypoints and gripper waypoints
        self.waypoints = []
        self.gripper_waypoints = []
        self.next_state = "idle"

    def print_waypoints(self):
        print('JOINT WAYPOINTS:')
        print(str(self.waypoints))
        print('GRIPPER WAYPOINTS:')
        print(str(self.gripper_waypoints))
        self.next_state = "idle"

    def dance(self):
        # overwrite waypoints to dance waypoints
        self.waypoints = dance_waypoints
        self.gripper_waypoints = dance_gripper
        # execute dance
        self.next_state = 'execute'

    def manual(self):
        """!
        @brief      Manually control the rxarm
        """
        self.status_message = "State: Manual - Use sliders to control arm"
        self.current_state = "manual"

    def idle(self):
        """!
        @brief      Do nothing
        """
        self.status_message = "State: Idle - Waiting for input"
        self.current_state = "idle"

    def estop(self):
        """!
        @brief      Emergency stop disable torque.
        """
        self.status_message = "EMERGENCY STOP - Check rxarm and restart program"
        self.current_state = "estop"
        self.rxarm.disable_torque()

    def execute(self):
        """!
        @brief      Go through all waypoints
        TODO: Implement this function to execute a waypoint plan
              Make sure you respect estop signal
        """
        self.status_message = "State: Execute - Executing motion plan"
        
        # go through each joint position in waypoints
        # check current state and waypoints are not empty
        if self.current_state != "estop" and self.waypoints and self.gripper_waypoints and self.rxarm.initialized:

            # previous gripper state
            prev_gripper_open = self.gripper_waypoints[0]

            # initialize gripper
            if prev_gripper_open:
                self.rxarm.open_gripper()
            else:
                self.rxarm.close_gripper()

            for joint_positions, gripper_open in zip(self.waypoints, self.gripper_waypoints):
                # execute to next state

                # check for estop state change
                if self.current_state == "estop":
                    # call disable torque
                    self.rxarm.disable_torque()
                    break

                self.rxarm.set_positions(joint_positions)

                # check if we have changed gripper state
                if prev_gripper_open != gripper_open:
                    # change gripper state to new state
                    if gripper_open:
                        self.rxarm.open_gripper()
                    else:
                        self.rxarm.close_gripper()

                # overwrite old gripper open bool
                prev_gripper_open = gripper_open

        elif self.current_state == 'estop':
            # disable torque as estop has occured
            print('ESTOP TRIGGERED!')
            self.rxarm.disable_torque()
        elif not self.waypoints or not self.gripper_waypoints:
            # waypoints are empty and nothing to execute
            print('WAYPOINTS ARE EMPTY!')
        elif not self.rxarm.initialized:
            # rxarm not initialized yet
            print('RXARM NOT INITIALIZED!')
        # set state to idle
        self.next_state = "idle"

    def calibrate(self):
        """!
        @brief      Gets the user input to perform the calibration
        """
        self.current_state = "calibrate"
        result = self.camera.calibrate()
        if result: 
            print('CAMERA CALIBRATED')
        else: 
            print('CAMERA DID NOT CALIBRATE')
        self.next_state = "idle"
        self.status_message = "Calibration - Completed Calibration"

    """ TODO """
    def detect(self):
        """!
        @brief      Detect the blocks
        """
        rospy.sleep(1)

    def initialize_rxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.current_state = "initialize_rxarm"
        self.status_message = "RXArm Initialized!"
        if not self.rxarm.initialize():
            print('Failed to initialize the rxarm')
            self.status_message = "State: Failed to initialize the rxarm!"
            rospy.sleep(5)
        self.next_state = "idle"


class StateMachineThread(QThread):
    """!
    @brief      Runs the state machine
    """
    updateStatusMessage = pyqtSignal(str)
    
    def __init__(self, state_machine, parent=None):
        """!
        @brief      Constructs a new instance.

        @param      state_machine  The state machine
        @param      parent         The parent
        """
        QThread.__init__(self, parent=parent)
        self.sm=state_machine

    def run(self):
        """!
        @brief      Update the state machine at a set rate
        """
        while True:
            self.sm.run()
            self.updateStatusMessage.emit(self.sm.status_message)
            rospy.sleep(0.05)