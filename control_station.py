#!/usr/bin/python
"""!
Main GUI for Arm lab
"""
import os
script_path = os.path.dirname(os.path.realpath(__file__))
from comp2 import comp
import argparse
import sys
import cv2
import numpy as np
import rospy
import time
from functools import partial
from copy import copy
import math

from PyQt4.QtCore import (QThread, Qt, pyqtSignal, pyqtSlot, QTimer)
from PyQt4.QtGui import (QPixmap, QImage, QApplication, QWidget, QLabel, QMainWindow, QCursor, QFileDialog)

from ui import Ui_MainWindow
from rxarm import RXArm, RXArmThread
from camera import Camera, VideoThread
from state_machine import StateMachine, StateMachineThread


""" Radians to/from  Degrees conversions """
D2R = np.pi / 180.0
R2D = 180.0 / np.pi

class Gui(QMainWindow):
    """!
    Main GUI Class

    Contains the main function and interfaces between the GUI and functions.
    """

    def __init__(self, station, parent=None, dh_config_file=None):
        QWidget.__init__(self,parent)
        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        """ Groups of ui commonents """
        self.joint_readouts = [
            self.ui.rdoutBaseJC,
            self.ui.rdoutShoulderJC,
            self.ui.rdoutElbowJC,
            self.ui.rdoutWristAJC,
            self.ui.rdoutWristRJC,
        ]
        self.joint_slider_rdouts = [
            self.ui.rdoutBase,
            self.ui.rdoutShoulder,
            self.ui.rdoutElbow,
            self.ui.rdoutWristA,
            self.ui.rdoutWristR,
        ]
        self.joint_sliders = [
            self.ui.sldrBase,
            self.ui.sldrShoulder,
            self.ui.sldrElbow,
            self.ui.sldrWristA,
            self.ui.sldrWristR,
        ]
        
        """Objects Using Other Classes"""
        print('Setting up camera station parameters')
        self.camera = Camera(station)
        
        print("Creating rx arm...")
        if(dh_config_file is not None):
            self.rxarm = RXArm(dh_config_file=dh_config_file)
        else:
            self.rxarm = RXArm()
        print("Done creating rx arm instance.")
        self.sm = StateMachine(self.rxarm, self.camera)

        """
        Attach Functions to Buttons & Sliders
        TODO: NAME AND CONNECT BUTTONS AS NEEDED
        """
        # Video
        self.ui.videoDisplay.setMouseTracking(True)
        self.ui.videoDisplay.mouseMoveEvent = self.trackMouse
        self.ui.videoDisplay.mousePressEvent = self.calibrateMousePress
        
        # Buttons
        # Handy lambda function that can be used with Partial to only set the new state if the rxarm is initialized
        #nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state if self.rxarm.initialized else None)
        nxt_if_arm_init = lambda next_state: self.sm.set_next_state(next_state)
        self.ui.btn_estop.clicked.connect(self.estop)
        self.ui.btn_init_arm.clicked.connect(self.initRxarm)
        self.ui.btn_torq_off.clicked.connect(lambda : self.rxarm.disable_torque())
        self.ui.btn_torq_on.clicked.connect(lambda : self.rxarm.enable_torque())
        self.ui.btn_sleep_arm.clicked.connect(lambda : self.rxarm.sleep())
        
        #User Buttons
        self.ui.btnUser1.setText("Calibrate")
        self.ui.btnUser1.clicked.connect(partial(nxt_if_arm_init, 'calibrate'))
        self.ui.btnUser2.setText('Open Gripper')
        self.ui.btnUser2.clicked.connect(lambda : self.rxarm.open_gripper())
        self.ui.btnUser3.setText('Close Gripper')
        self.ui.btnUser3.clicked.connect(lambda : self.rxarm.close_gripper())
        self.ui.btnUser4.setText('Execute')
        self.ui.btnUser4.clicked.connect(partial(nxt_if_arm_init, 'execute'))
        
        # # teach
        self.ui.btnUser5.setText('Save State Open')
        # append state to waypoints list
        self.ui.btnUser5.clicked.connect(lambda : self.sm.set_next_state('append_state_open'))
        
        # teach
        self.ui.btnUser6.setText('Save State Closed')
        # append state to waypoints list
        self.ui.btnUser6.clicked.connect(lambda : self.sm.set_next_state('append_state_closed'))

        # Start recording EE
        self.ui.btnUser12.setText('Begin EE Record')
        # append state to waypoints list
        self.ui.btnUser12.clicked.connect(lambda : self.sm.set_next_state('record_ee'))
        
        # End recording EE
        # self.ui.btnUser6.setText('Save EE Record')
        # # append state to waypoints list
        # self.ui.btnUser6.clicked.connect(lambda : self.sm.set_next_state('save_ee'))




        # Clear Waypoints
        self.ui.btnUser7.setText('Reset Waypoints')
        # append state to waypoints list
        self.ui.btnUser7.clicked.connect(lambda : self.sm.reset_waypoints())

        # Print Waypoints
        self.ui.btnUser8.setText('PICK')
        self.ui.btnUser8.clicked.connect(lambda : self.sm.set_next_state('pick'))

        # Dance 
        self.ui.btnUser9.setText('PLACE')
        self.ui.btnUser9.clicked.connect(lambda : self.sm.set_next_state('place'))

        # Dance 
        self.ui.btnUser10.setText('DANCE BABY!')
        self.ui.btnUser10.clicked.connect(lambda : self.sm.dance())

        # Find Blocks 
        self.ui.btnUser11.setText('Find Blocks')
        self.ui.btnUser11.clicked.connect(lambda : self.camera.detectAll())

        # comp 2
        self.ui.btn_task1.clicked.connect(lambda : comp(self.rxarm, self.camera, self.sm, 1))
        self.ui.btn_task2.clicked.connect(lambda : comp(self.rxarm, self.camera, self.sm, 2))
        self.ui.btn_task3.clicked.connect(lambda : self.comp3())

        # Sliders
        for sldr in self.joint_sliders:
            sldr.valueChanged.connect(self.sliderChange)
        self.ui.sldrMoveTime.valueChanged.connect(self.sliderChange)
        self.ui.sldrAccelTime.valueChanged.connect(self.sliderChange)
        # Direct Control
        self.ui.chk_directcontrol.stateChanged.connect(self.directControlChk)
        # Status
        self.ui.rdoutStatus.setText("Waiting for input")

        """initalize manual control off"""
        self.ui.SliderFrame.setEnabled(False)

        """Setup Threads"""

        # State machine
        self.StateMachineThread = StateMachineThread(self.sm)
        self.StateMachineThread.updateStatusMessage.connect(self.updateStatusMessage)
        self.StateMachineThread.start()
        self.VideoThread = VideoThread(self.camera)
        self.VideoThread.updateFrame.connect(self.setImage)
        self.VideoThread.start()
        self.ArmThread = RXArmThread(self.rxarm)
        self.ArmThread.updateJointReadout.connect(self.updateJointReadout)
        self.ArmThread.updateEndEffectorReadout.connect(self.updateEndEffectorReadout)
        self.ArmThread.start()

    """ Slots attach callback functions to signals emitted from threads"""

    @pyqtSlot(str)
    def updateStatusMessage(self, msg):
        self.ui.rdoutStatus.setText(msg)

    @pyqtSlot(list)
    def updateJointReadout(self, joints):
        for rdout, joint in zip(self.joint_readouts, joints):
            rdout.setText(str('%+.2f' % (joint * R2D)))
    
    ### TODO: output the rest of the orientation according to the convention chosen
    @pyqtSlot(list)
    def updateEndEffectorReadout(self, pos):
        self.ui.rdoutX.setText(str("%+.2f mm" % (1000*pos[0])))
        self.ui.rdoutY.setText(str("%+.2f mm" % (1000*pos[1])))
        self.ui.rdoutZ.setText(str("%+.2f mm" % (1000*pos[2])))
        self.ui.rdoutPhi.setText(str("%+.2f rad" % (pos[3])))
        self.ui.rdoutTheta.setText(str("%+.2f" % (pos[4])))
        self.ui.rdoutPsi.setText(str("%+.2f" % (pos[5])))
        # for recording data 
        if self.sm.record_endeffector_position:
            # append time
            pos.append(time.time())
            # append to list of measurements
            self.sm.endeffector_positions.append(pos)


    @pyqtSlot(QImage, QImage, QImage)
    def setImage(self, rgb_image, depth_image, tag_image):
        """!
        @brief      Display the images from the camera.

        @param      rgb_image    The rgb image
        @param      depth_image  The depth image
        """
        if(self.ui.radioVideo.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(rgb_image))
        if(self.ui.radioDepth.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(depth_image))
        if(self.ui.radioUsr1.isChecked()):
            self.ui.videoDisplay.setPixmap(QPixmap.fromImage(tag_image))

    """ Other callback functions attached to GUI elements"""

    def estop(self):
        self.rxarm.disable_torque()
        self.sm.set_next_state('estop')

    def sliderChange(self):
        """!
        @brief Slider changed

        Function to change the slider labels when sliders are moved and to command the arm to the given position
        """
        for rdout, sldr in zip(self.joint_slider_rdouts, self.joint_sliders):
            rdout.setText(str(sldr.value()))

        self.ui.rdoutMoveTime.setText(str(self.ui.sldrMoveTime.value()/10.0) + "s")
        self.ui.rdoutAccelTime.setText(str(self.ui.sldrAccelTime.value()/20.0) + "s")
        self.rxarm.set_moving_time(self.ui.sldrMoveTime.value()/10.0)
        self.rxarm.set_accel_time(self.ui.sldrAccelTime.value()/20.0)

        # Do nothing if the rxarm is not initialized
        if self.rxarm.initialized:
            joint_positions = np.array([sldr.value() * D2R for sldr in self.joint_sliders])
            # Only send the joints that the rxarm has
            self.rxarm.set_positions(joint_positions[0:self.rxarm.num_joints])


    def getHighestBlock(self):
        depth = -10000
        color_key = ''
        for color in self.camera.colorBases.keys():
            if self.camera.blockState[color]['pixel'][2] == 0:
                continue
            
            color_depth = self.camera.blockState[color]['position'][2]
            
            if color_depth > depth and color_depth < 0.20:
                depth = self.camera.blockState[color]['position'][2]
                color_key = color
        if color_key == '':
            return False, color_key
        return True, color_key
            
    def pixel2coordinates(self, point): 
        # take in a pixel near a block and get the block world frame coordiantes back 
        # this can then be fed into PICK/PLACE
        self.camera.last_click = (point[0], point[1])
        self.camera.blockDetector()
        pickCords = self.camera.pointToWorld(self.camera.block_detections)
        return pickCords


    def color2coordinates(self, color, color_vec, initial_coordinates, offset = 0.025, axis='x'):
        count = -1
        for color_key in color_vec:
            count += 1
            if color_key == color:
                break
        
        new_coordinates = initial_coordinates
        if axis=='x':
            new_coordinates[0] += offset*count
        elif axis == 'y':
            new_coordinates[1] += offset*count
        elif axis == 'z':
            new_coordinates[2] += offset*count
        
        return new_coordinates

    def comp3(self):
        # sleep arm before detecting blocks
        self.rxarm.sleep()
        # Detect all blocks
        time.sleep(5)

        # reset blockstate pixel locations (when running multiple times)
        self.camera.resetBlockState()
        
        # detect all blocks multiple times
        for i in range(15):
            self.camera.detectAll()
        
        # order that blocks need to be processed
        colors = self.camera.colorBases.keys()
        
        print('WILL PICK IN THIS ORDER:')
        print(colors)
        # colors = ['Black', 'Blue', 'Pink']
        # upper left hand side
        
        
        # where to put first block
        initial_pixel = [186, 158]
        initial_coordinates = self.camera.pointToWorld(initial_pixel)
        
        # where to put blocks in limbo 
        destack_pixel = [205, 420]
        destack_coordinates = self.camera.pointToWorld(destack_pixel)


        # initial pixel
        for id, color in enumerate(colors):
            # sleep arm before detecting blocks
            self.rxarm.sleep()
            # Detect all blocks
            time.sleep(5)
            # detect all blocks multiple times
            for i in range(10):
                self.camera.detectAll()

            # check if current color has been found 
            pixel = self.camera.blockState[color]['pixel']
            
            # check if current color is not available to be ordered (under a stack)
            if not pixel[2]:
                # continue to destack until we see the color
                while not pixel[2]:
                    # find color to destack
                    success, deStackColor = self.getHighestBlock()
                    print('{color1} NOT FOUND... Destacking {deStackColor1}'.format(color1=color, deStackColor1=deStackColor))
                    print(self.camera.blockState[deStackColor])
                    if success:
                        # get the pixel coordinates 
                        deStackPixel = self.camera.blockState[deStackColor]['pixel']
                        # pickup at pixel location
                        pick = self.pixel2coordinates(deStackPixel)
                        # place at limbo location
                        placeCoordinates = self.color2coordinates(deStackColor, colors,
                                                                destack_coordinates,
                                                                axis='y')
                        place = placeCoordinates
                        
                        # initialize
                        self.rxarm.initialize()
                        time.sleep(5)

                        # pick up and place found block
                        self.sm.pick(pick)  
                        time.sleep(10)
                        self.sm.place(place)
                        time.sleep(10)

                        # sleep arm before detecting blocks
                        self.rxarm.sleep()
                        # Detect all blocks
                        time.sleep(10)
                        for i in range(5):
                            self.camera.detectAll()
                        self.rxarm.initialize()
                        time.sleep(5)
                    print('{color1} STATUS W/DE-STACK'.format(color1=color))
                    print(self.camera.blockState[color])
                    # reset pixel
                    pixel = self.camera.blockState[color]['pixel']
            
            # should have desired color exposed to move to correct location
            if pixel[2] and (not id):
                # Found first color
                # go to initial position
                pick = self.pixel2coordinates(pixel)
                place = initial_coordinates 
            else:
                # go to a specific location (below the last color that was used)
                pick = self.pixel2coordinates(pixel)
                # initialize
                self.rxarm.initialize()
                time.sleep(4)
                print('PICKING {color1} AT {loc}'.format(color1=color,loc=pick))
                # pick up and place in correct location
                self.sm.pick(pick)
                time.sleep(10)
      
                # find new block positions
                self.camera.detectAll()
                time.sleep(0.5)
                self.camera.detectAll()
                time.sleep(0.5)
                self.camera.detectAll()
                
                min_dist = 1000000.0
                min_pix = initial_pixel
                min_color = ''
                for key in self.camera.blockState.keys():
                    pix = self.camera.blockState[key]['pixel']
                    if pix[2] != 0:
                        dist = math.sqrt((initial_pixel[0] - pix[0])**2 + (initial_pixel[1] - pix[1])**2)
                        if dist < min_dist:
                            min_color = copy(key)
                            min_pix = copy(pix)
                            min_dist = dist
                print('USING {color1} FOUND PIXEL:'.format(color1=min_color))

                # place_pixel = self.camera.blockState[colors[i-1]]['pixel']

                max_pix_radius = 5
                placed_block = False
                for i in [0,-1, 1]: #, 2, -2, 3, -3]:
                    for j in [0, -1, 1]: #, 1, 2, -2, 3, -3]:
                        if placed_block:
                            break
                        for k in range(1, max_pix_radius):
                            self.camera.last_click = (min_pix[0] + k*i, min_pix[1] + k*j)
                            self.camera.blockDetector()
                            placeCords = self.camera.pointToWorld(self.camera.block_detections)

                            if self.sm.place(placeCords) and not placed_block:
                                print('PLACING {color} BLOCK'.format(color=color))
                                print('INDEX:', k*i, k*j)
                                placed_block = True 
                                break
            time.sleep(10)
 
        return


    def comp2(self):
        # sleep arm before detecting blocks
        self.rxarm.sleep()
        # Detect all blocks
        time.sleep(5)

        # reset blockstate pixel locations (when running multiple times)
        self.camera.resetBlockState()
        
        # detect all blocks multiple times
        for i in range(15):
            self.camera.detectAll()
        
        # order that blocks need to be processed
        colors = self.camera.colorBases.keys()
        
        print('WILL PICK IN THIS ORDER:')
        print(colors)
        # colors = ['Black', 'Blue', 'Pink']
        # upper left hand side
        
        
        # where to put first block
        initial_pixel = [186, 158]
        initial_coordinates = self.camera.pointToWorld(initial_pixel)
        
        # where to put blocks in limbo 
        destack_pixel = [205, 420]
        destack_coordinates = self.camera.pointToWorld(destack_pixel)


        # initial pixel
        for id, color in enumerate(colors):
            # sleep arm before detecting blocks
            self.rxarm.sleep()
            # Detect all blocks
            time.sleep(5)
            # detect all blocks multiple times
            for i in range(10):
                self.camera.detectAll()

            # check if current color has been found 
            pixel = self.camera.blockState[color]['pixel']
            
            # check if current color is not available to be ordered (under a stack)
            if not pixel[2]:
                # continue to destack until we see the color
                while not pixel[2]:
                    # find color to destack
                    success, deStackColor = self.getHighestBlock()
                    print('{color1} NOT FOUND... Destacking {deStackColor1}'.format(color1=color, deStackColor1=deStackColor))
                    print(self.camera.blockState[deStackColor])
                    if success:
                        # get the pixel coordinates 
                        deStackPixel = self.camera.blockState[deStackColor]['pixel']
                        # pickup at pixel location
                        pick = self.pixel2coordinates(deStackPixel)
                        # place at limbo location
                        placeCoordinates = self.color2coordinates(deStackColor, colors,
                                                                destack_coordinates,
                                                                axis='y')
                        place = placeCoordinates
                        
                        # initialize
                        self.rxarm.initialize()
                        time.sleep(5)

                        # pick up and place found block
                        self.sm.pick(pick)  
                        time.sleep(10)
                        self.sm.place(place)
                        time.sleep(10)

                        # sleep arm before detecting blocks
                        self.rxarm.sleep()
                        # Detect all blocks
                        time.sleep(10)
                        for i in range(5):
                            self.camera.detectAll()
                        self.rxarm.initialize()
                        time.sleep(5)
                    print('{color1} STATUS W/DE-STACK'.format(color1=color))
                    print(self.camera.blockState[color])
                    # reset pixel
                    pixel = self.camera.blockState[color]['pixel']
            
            # should have desired color exposed to move to correct location
            if pixel[2] and (not id):
                # Found first color
                # go to initial position
                pick = self.pixel2coordinates(pixel)
                place = initial_coordinates 
            else:
                # go to a specific location (below the last color that was used)
                pick = self.pixel2coordinates(pixel)
                place = self.color2coordinates(color, colors, initial_coordinates)
            
            # initialize
            self.rxarm.initialize()
            time.sleep(4)
            print('PICKING {color1} AT {loc}'.format(color1=color,loc=pick))
            # pick up and place in correct location
            self.sm.pick(pick)
            time.sleep(10)
            print('PLACING {color1} AT {loc}'.format(color1=color,loc=place))
            self.sm.place(place)
            time.sleep(10)
 
        return


    def comp1(self):
        if not self.camera.cameraCalibrated or not self.rxarm.initialized:
            print('CAMERA/ARM NOT INITIALIZED!')
            return
        # get all colors needed for competition
        colors = ['Red', 'Green', 'Blue']
        self.current_state = 'execute'
        # get all block states
        
        
        
        self.camera.detectAll()
        self.camera.detectAll()
        self.camera.detectAll()

        # go through all colors in block states and mark them as incomplete
        for color in self.camera.blockState.keys():
            self.camera.blockState[color]['complete'] = False
        
        # check if blocks are at valid initial state
        initialized = True
        
        # go through colors in comp and make sure we can see them
        for color in colors:
            if self.camera.blockState[color]['pixel'][0] == 0:
                # raise error
                print('{color} WAS NOT FOUND! EXITING'.format(color=color))
                return
            if self.camera.blockState[color]['pixel'][1] > 0:
                initialized = False
        
        # check if the robot is not initalized
        if not initialized:
            print('BLOCKS ARE NOT ON BOARD/CORRECT SIDE')

        # pick a spot on the other side of the board to initialize first block to
        initial_pixel = [230, 230]
        # find specific coordinates
        initialCords = self.camera.pointToWorld(initial_pixel)

        # go through all blocks and move them to the other side
        
        for i, color in enumerate(colors):
            pick_point = self.camera.blockState[color]['pixel']
            # set the last click

            self.camera.last_click = (pick_point[0], pick_point[1])
            self.camera.blockDetector()

            pickCords = self.camera.pointToWorld(self.camera.block_detections)
            
            
            print('ABOUT TO PICK UP {color} BLOCK!'.format(color=color))
            if self.sm.pick(pickCords):
                time.sleep(10)
                # time.sleep(self.rxarm.get_total_time()+1)
                print('PICKED UP {color} BLOCK!'.format(color=color))
            else:
                print('FAILED TO PICKUP {color}'.format(color=color))
            
            # 
            if (not i):
                print('PLACING {color} BLOCK'.format(color=color))
                self.sm.place(initialCords)
            else:
                # find new block positions
                self.camera.detectAll()
                time.sleep(0.5)
                self.camera.detectAll()
                time.sleep(0.5)
                self.camera.detectAll()
                
                min_dist = 1000000.0
                min_pix = initial_pixel
                min_color = ''
                for key in self.camera.blockState.keys():
                    pix = self.camera.blockState[key]['pixel']
                    if pix[2] != 0:
                        dist = math.sqrt((initial_pixel[0] - pix[0])**2 + (initial_pixel[1] - pix[1])**2)
                        if dist < min_dist:
                            min_color = copy(key)
                            min_pix = copy(pix)
                            min_dist = dist
                print('USING {color1} FOUND PIXEL:'.format(color1=min_color))

                # place_pixel = self.camera.blockState[colors[i-1]]['pixel']

                max_pix_radius = 5
                placed_block = False
                for i in [0,-1, 1]: #, 2, -2, 3, -3]:
                    for j in [0, -1, 1]: #, 1, 2, -2, 3, -3]:
                        if placed_block:
                            break
                        for k in range(1, max_pix_radius):
                            self.camera.last_click = (min_pix[0] + k*i, min_pix[1] + k*j)
                            self.camera.blockDetector()
                            placeCords = self.camera.pointToWorld(self.camera.block_detections)

                            if self.sm.place(placeCords) and not placed_block:
                                print('PLACING {color} BLOCK'.format(color=color))
                                print('INDEX:', k*i, k*j)
                                placed_block = True 
                                break
                
            # sleep for moving time
            time.sleep(10)
            # time.sleep(self.rxarm.get_total_time()+1)
            
            # go to initialize state at the end
        self.sm.next_state = 'initialize_rxarm'


    def directControlChk(self, state):
        """!
        @brief      Changes to direct control mode

                    Will only work if the rxarm is initialized.

        @param      state  State of the checkbox
        """
        if state == Qt.Checked and self.rxarm.initialized:
            # Go to manual and enable sliders
            self.sm.set_next_state("manual")
            self.ui.SliderFrame.setEnabled(True)
        else:
            # Lock sliders and go to idle
            self.sm.set_next_state("idle")
            self.ui.SliderFrame.setEnabled(False)
            self.ui.chk_directcontrol.setChecked(False)

    def trackMouse(self, mouse_event):
        """!
        @brief      Show the mouse position in GUI

                    TODO: after implementing workspace calibration display the world coordinates the mouse points to in the RGB
                    video image.

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        pt = mouse_event.pos()
        if self.camera.DepthFrameRaw.any() != 0:
            z = self.camera.DepthFrameRaw[pt.y()][pt.x()]
            self.ui.rdoutMousePixels.setText("(%.0f,%.0f,%.0f)" % (pt.x(), pt.y(), z))
            worldpt = self.camera.pointToWorld([pt.x(), pt.y()])
            self.ui.rdoutMouseWorld.setText("(%.0f,%.0f,%.0f)" % (worldpt[0]*1000, worldpt[1]*1000, worldpt[2]*1000))

    def calibrateMousePress(self, mouse_event):
        """!
        @brief Record mouse click positions for calibration

        @param      mouse_event  QtMouseEvent containing the pose of the mouse at the time of the event not current time
        """

        """ Get mouse posiiton """
        pt = mouse_event.pos()
        self.camera.last_click = (pt.x(),pt.y())
        self.camera.new_click = True

        if self.sm.next_state == "pick":
            self.camera.blockDetector()
            pickCords = self.camera.pointToWorld(self.camera.block_detections)
            print('PICK:')
            print(pickCords)
            self.sm.pick(pickCords)
        if self.sm.next_state == "place":
            placeCords = self.camera.pointToWorld(self.camera.last_click)
            self.sm.place(placeCords)

    def initRxarm(self):
        """!
        @brief      Initializes the rxarm.
        """
        self.ui.SliderFrame.setEnabled(False)
        self.ui.chk_directcontrol.setChecked(False)
        self.rxarm.enable_torque()
        self.sm.set_next_state('initialize_rxarm')

### TODO: Add ability to parse POX config file as well
def main(args=None):
    """!
    @brief      Starts the GUI
    """
    app = QApplication(sys.argv)
    app_window = Gui(dh_config_file=args['dhconfig'], station=int(args['station']))
    app_window.show()
    sys.exit(app.exec_())


# Run main if this file is being run directly
### TODO: Add ability to parse POX config file as well
if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument("-c", "--dhconfig", required=False, help="path to DH parameters csv file")
    ap.add_argument("-s", "--station", required=True, help="GIVE VALUE OF 1-5")
    main(args=vars(ap.parse_args()))
