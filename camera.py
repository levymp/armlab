"""!
Class to represent the camera.
"""

import cv2
import time
import math
import numpy as np
from PyQt4.QtGui import QImage
from PyQt4.QtCore import QThread, pyqtSignal, QTimer
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from apriltag_ros.msg import *
from cv_bridge import CvBridge, CvBridgeError
from scipy.spatial.transform import Rotation as R
from copy import copy
import os
import pandas as pd
from kinematics import clamp

class Camera():
    """!
    @brief      This class describes a camera.
    """

    def __init__(self, station):
        """!
        @brief      Constructs a new instance.
        """
        self.VideoFrame = np.zeros((480,640,3)).astype(np.uint8)
        self.MaskFrame = np.zeros((480,640,3)).astype(np.uint8)
        self.TagImageFrame = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRaw = np.zeros((480,640)).astype(np.uint16)
        """ Extra arrays for colormaping the depth image"""
        self.DepthFrameHSV = np.zeros((480,640,3)).astype(np.uint8)
        self.DepthFrameRGB=np.array([])

        # mouse clicks & calibration variables
        self.depth2rgb_affine = np.float32([[1,0,0],[0,1,0]])
        self.cameraCalibrated = False
        self.intrinsic_matrix = np.array([])
        self.distortion = np.array([])
        self.last_click = np.array([0,0])
        self.new_click = False
        self.rgb_click_points = np.zeros((5,2),int)
        self.depth_click_points = np.zeros((5,2),int)
        self.tag_detections = np.array([])
        self.not_calibrate_msg = False
        # AR Tag information
        self.ar_tag_detections = [[],[],[],[]]
        self.tag_1_wf_pose = np.array([[-.1416], [0], [-.06691], [0]])
        self.rgb2world = None
        self.color = "None"
        self.colorBases = {

          'Red': [70, 60, 100],
          'Green': [100,100,10],
          'Blue': [150, 60, 0], 
          'Violet': [118,  52,  59],
          'Black': [50,50,20],
          'Yellow': [20,190,190],
          'Pink': [120,70,160],
          'Orange': [25,84,200]}
        """ block info """
        self.block_contours = np.array([])
        self.block_detections = np.array([0,0])
        self.blockState = {'Blue': {'pixel':[0,0,0]}, 
          'Red': {'pixel':[0,0,0]},
          'Green': {'pixel':[0,0,0]},
          'Black': {'pixel':[0,0,0]},
          'Yellow': {'pixel':[0,0,0]},
          'Violet': {'pixel':[0,0,0]},
          'Pink': {'pixel':[0,0,0]},
          'Orange': {'pixel':[0,0,0]}}
        # get work station dimensions and find all world coordinates of each april tag
        df = pd.read_csv('dimensions.csv', index_col=0).T
        tableDimensions = df.iloc[station-1]
        self.tag_locations = self.set_tag_locations(tableDimensions)


    def set_tag_locations(self, td):
      w_tags = np.matrix(np.ones((4,4)))

      # all in (mm)
      # tag 1 
      w_tags[:,0] = self.tag_1_wf_pose * 1000
      w_tags[3,0] = 1

      # tag 2
      w_tags[0,1] = -(td['World Offset y'] - td['T2 Y'])
      w_tags[1,1] = td['World Offset X'] - td['T2 X']

      # tag 3
      w_tags[0,2] = td['Board Length'] - td['World Offset y'] - td['T4 Y']
      w_tags[1,2] = td['World Offset X'] - td['T3 X']

      # tag 4
      w_tags[0,3] = -(td['World Offset y'] - td['T4 Y'])
      w_tags[1,3] = -(td['Board Width'] - td['World Offset X'] - td['T4 X'])

      # set 2-4 depths
      w_tags[2,1:] = -103.91

      # convert to meters
      w_tags = w_tags * .001

      w_tags[3,:] = 1
      

      return w_tags
      

    
    def processVideoFrame(self):
        """!
        @brief      Process a video frame
        """
        cv2.drawContours(self.VideoFrame, self.block_contours, -1, (255,0,255))
        #cv2.putText(self.VideoFrame, self.color, (self.block_detections[0],self.block_detections[1]), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        for c in self.colorBases:
          blockPixel = self.blockState[c]['pixel']
          if blockPixel[0] > 0:
            cv2.circle(self.TagImageFrame, (int(blockPixel[0]), int(blockPixel[1])), 4, (0,0,0), -1)
            cv2.putText(self.TagImageFrame, c, (int(blockPixel[0]),int(blockPixel[1])), cv2.FONT_HERSHEY_SIMPLEX, .5, (255,255,255), 1)
        
        #self.VideoFrame = self.MaskFrame
    def ColorizeDepthFrame(self):
        """!
        @brief Converts frame to colormaped formats in HSV and RGB
        """
        self.DepthFrameHSV[...,0] = self.DepthFrameRaw
        self.DepthFrameHSV[...,1] = 0x9F
        self.DepthFrameHSV[...,2] = 0xFF
        self.DepthFrameRGB = cv2.cvtColor(self.DepthFrameHSV,cv2.COLOR_HSV2RGB)

    def loadVideoFrame(self):
        """!
        @brief      Loads a video frame.
        """
        self.VideoFrame = cv2.cvtColor(
            cv2.imread("data/rgb_image.png",cv2.IMREAD_UNCHANGED),cv2.COLOR_BGR2RGB)

    def loadDepthFrame(self):
        """!
        @brief      Loads a depth frame.
        """
        self.DepthFrameRaw = cv2.imread("data/raw_depth.png",0).astype(np.uint16)

    def convertQtVideoFrame(self):
        """!
        @brief      Converts frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.VideoFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def convertQtDepthFrame(self):
       """!
       @brief      Converts colormaped depth frame to format suitable for Qt

       @return     QImage
       """
       try:
           img = QImage(self.DepthFrameRGB,
                            self.DepthFrameRGB.shape[1],
                            self.DepthFrameRGB.shape[0],
                            QImage.Format_RGB888
                            )
           return img
       except:
           return None

    def convertQtTagImageFrame(self):
        """!
        @brief      Converts tag image frame to format suitable for Qt

        @return     QImage
        """

        try:
            frame = cv2.resize(self.TagImageFrame, (640, 480))
            img = QImage(frame,
                             frame.shape[1],
                             frame.shape[0],
                             QImage.Format_RGB888
                             )
            return img
        except:
            return None

    def calibrate(self):
      # check that enough detections have been found for the four tags
      for detection_list in self.ar_tag_detections:
        if len(detection_list) != 100:
          return False  
        
      
            
      # position and orientation will be average of the last ten AR Tag Detections
      tags = []
      
      # go through all detections and add them to the position/orientation
      for detection_list in self.ar_tag_detections:
        position = np.array([0.0, 0.0, 0.0])
        orientation = np.array([0.0, 0.0, 0.0, 0.0])
        for detection in detection_list:
          for i in range(4):
            if i < 3:
              # position is length 3
              position[i] += detection['position'][i] 
            # orientation is length 4
            orientation[i] += detection['orientation'][i]

        position /= 100
        orientation /= 100

        # calculate new depth based on depth camera
        tagDetectionZ = position[2]      
        # find pixel location
        tag_pixel = (1/tagDetectionZ) * np.matmul(self.intrinsic_matrix, position)
        # from pixel location find depth
        depthZ = float(self.DepthFrameRaw[int(tag_pixel[1]), int(tag_pixel[0])]) * .001
        position[2] = depthZ
        # tag_pixel = (1/position[2]) * np.matmul(self.intrinsic_matrix, position)

        # save to list
        tags.append(copy({'position': position, 'orientation': orientation, 'pixel': tag_pixel[0:2]}))
      
      # # tags contains a list of position/orientation for each tag (length 4)
      # tag_model_points = np.array([tag['position'] for tag in tags])
      # # tag_model_points -= tag_model_points[0,:]
      tag_image_points = np.array([tag['pixel'] for tag in tags])
      print("IMG", tag_image_points)
      # print("TAG MDOEL", tag_model_points)

      relative_positions = self.tag_locations
      model_points = np.asarray(relative_positions.T[:,:3])

      print("MODEL POINTS", model_points)
      # pixel -> camera
      (success, r_vec, t_vec) = cv2.solvePnP(model_points, tag_image_points, self.intrinsic_matrix, self.distortion, cv2.SOLVEPNP_ITERATIVE)

      for tag in tags:
            print(tag['position'])
      # camera -> AR TAG -> world

      print("PnP Success = ", success)
      r_mtx = np.zeros((3,3))
      cv2.Rodrigues(r_vec, r_mtx)
      # r_mtx_camera2world = (r_mtx - r_mtx.T)/2
      
      # rotate from camera -> AR Tag Frame
      # rotation = R.from_quat(tags[0]['orientation'])
      # print('EULER FROM AR: {rotation}'.format(rotation=rotation.as_euler('xyz', degrees=True)))
      # rotation from AR Tag Frame -> World Frame
      # re = rotation.as_euler('xyz', degrees=True)
      # re[0] = 179
      # re[1] = 0
      # re[2] -= 93
      
      # rnew = R.from_euler('xyz', re, degrees=True)

      # set rotation matrix (old approach)
      # rotation_ttw = R.from_euler('xz', [180,90], degrees=True)
      # rotation_ttw = rotation_ttw.as_dcm()

      # rotation = rnew.as_dcm()
      # print("rotation prev shape:", rotation)
      # print("new rotation shape:", r_mtx_camera2world)
      # rotation from World -> Camera Frame
      # rotation = np.matmul(rotation_ttw, rotation)
      # get translation from pose of april-tag
      # translation = tags[0]['position']
      
      # print('translation prev shape:')
      # print(translation)
      # print('new translation shape:')
      # print(t_vec)
      
      
      
      # # form Transform
      # holo = np.hstack((rotation, np.transpose([translation])))
      # self.rgb2world = np.vstack((holo, np.array([0, 0, 0, 1])))
      # # inverse
      # self.rgb2world = np.linalg.inv(self.rgb2world)

      
      # add in translation
      # print('before')
      # print(self.rgb2world)
      # self.rgb2world = self.rgb2world + np.hstack((np.zeros((4,3)), self.tag_1_wf_pose))
      # print(self.rgb2world)
      # # compare rotation/translation here!
      # r = R.from_matrix(self.rgb2world)
      # rgb_copy = copy(self.rgb2world)

      new_rotation = np.vstack((np.hstack((r_mtx, t_vec)) , np.array([0, 0, 0, 1])))
      new_rotation = np.linalg.inv(new_rotation)
      
      self.cameraCalibrated = True

      for tag_id, tag in enumerate(tags):
        print('TAG ', tag_id)
        self.rgb2world = new_rotation
        world_loc = self.pointToWorld(tag['pixel'])  
        print('TAG CAL:')
        print(world_loc * 1000)

      #   print('------------------------------------------------')
      #   print('------------------------------------------------')
      self.rgb2world = new_rotation
      return True
    

    def getAffineTransform(self, coord1, coord2):
        """!
        @brief      Find the affine matrix transform between 2 sets of corresponding coordinates.

                    TODO: Rewrite this function to take in an arbitrary number of coordinates and find the transform without
                    using cv2 functions

        @param      coord1  The coordinate 1
        @param      coord2  The coordinate 2

        @return     Affine transform between coordinates.
        """
        pts1 = coord1[0:3].astype(np.float32)
        pts2 = coord2[0:3].astype(np.float32)
        return cv2.getAffineTransform(pts1, pts2)


    def registerDepthFrame(self, frame):
        """!
        @brief      Transform the depth frame to match the RGB frame

                    TODO: Using an Affine transformation, transform the depth frame to match the RGB frame using
                    cv2.warpAffine()

        @param      frame  The frame

        @return     { description_of_the_return_value }
        """
        return frame

    def pointToWorld(self, pt):
        # check if calibrated
        if not self.cameraCalibrated:
          if not self.not_calibrate_msg:
            print('CAMERA NOT CALIBRATED!')
            # already sent a message
            self.not_calibrate_msg = True
          return np.zeros((4,1))
        
        # calculate the depth
        depth = float(self.DepthFrameRaw[int(pt[1]), int(pt[0])]) * .001
        
        # set pixels pixels
        pt = np.array([[pt[0]],[pt[1]],[1]])

        # calculate the camera frame 4x1
        cameraframe = depth*np.matmul(np.linalg.inv(self.intrinsic_matrix), pt)
        #cameraframe[2] = depth
        cameraframe = np.vstack((cameraframe, np.array([1])))
        
        # calculate the world frame
        world = np.matmul(self.rgb2world, cameraframe)
        return world
    
    def detectAll(self):
        # current rgb image
        rgbimg = self.VideoFrame
        depthimg = self.DepthFrameRaw
        h,w,c = rgbimg.shape
        mask = cv2.inRange(depthimg, (0), (950))
        for i in range(h):
              for j in range(w):
                    self.MaskFrame[i,j,0] = mask[i,j]
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mcs = np.array([0,0])
        for c in range(len(contours)):
          mu = cv2.moments(contours[c])
          mid = (mu['m10'] / (mu['m00'] + 1e-5), mu['m01'] / (mu['m00'] + 1e-5))
          self.last_click = (int(mid[0]),int(mid[1]))
          self.blockDetector()
          mc = self.block_detections
          newbox = self.block_contours[0]
          color = self.color
          blockside1 = math.sqrt((newbox[0][0]-newbox[1][0])**2+(newbox[0][1]-newbox[1][1])**2) 
          blockside2 = math.sqrt((newbox[2][0]-newbox[1][0])**2+(newbox[2][1]-newbox[1][1])**2) 
          if (blockside1 > 10) & (blockside1 < 40) & (blockside2 > 10) & (blockside2 < 40):
            mcs = np.vstack((mcs, mc))
            self.blockState[color]['pixel'] = [int(mc[0]),int(mc[1]), depthimg[int(mc[1]),int(mc[0])]]
            self.blockState[color]['position'] = self.pointToWorld(self.blockState[color]['pixel'])
    

      # TODO
    # rank all takes in all blocks and outputs 
 

      # priority queue of next block to grab?
        # prioritized differently based on competition

      # dictionary: Pixel (u, v, z)
      #             position (x,y,z)
                    # reachable?
                    # Color str
                    # Sorted bool

      # 

      # stateMachine Comp (order of blocks, stack?=Bool)
        # detectallf
        # if stack? 
        # use same pixel 
        # else: start at inital pixel and offset accordingly after each placement


      # stateMachine Event3
      
      
      # event 2 pick/stack
        # 3 blocks (unstacked) > 2.5 cm apart
        # stack on other side of board
        # 

    def blockDetector(self):
        """!
        @brief      Detect blocks from rgb

                    TODO: Implement your block detector here. You will need to locate blocks in 3D space and put their XYZ
                    locations in self.block_detections
        """

        # current rgb image
        rgbimg = self.VideoFrame
        
        # height/width/rgb shape
        h,w,c = rgbimg.shape

        # get pixel coordinate of last click
        point = self.last_click

        # find rgb image in opencv format
        blocks = cv2.cvtColor(rgbimg, cv2.COLOR_BGR2RGB)
        color = blocks[point[1],point[0], :]

        # average color surrounding point
        avgc = np.array([0,0,0])
        # average difference in color around a pixel
        adif = np.array([0,0,0])
        
        sq = 7
        for i in range(sq):
          i = int(i-(i-1)/2)
          for j in range(sq):
            j = int(j-(j-1)/2)
            if (point[1]+i < h) & (point[1]+i > 0) & (point[0]+j < w) & (point[0]+j > 0):
              avgc = avgc + blocks[point[1]+i,point[0]+j, :]
              adif = adif + np.subtract(blocks[point[1]+i,point[0]+j, :], color)

        avgc = avgc/(sq*sq)
        adif = 10+adif/(sq*sq*20)
        
        # range of colors block exists in
        colorlow = np.array([avgc[0]-(adif[0]), avgc[1]-(adif[1]), avgc[2]-(adif[2])])
        colorhigh = np.array([avgc[0]+(adif[0]), avgc[1]+(adif[1]), avgc[2]+(adif[2])])


        mask = cv2.blur(cv2.inRange(blocks, colorlow, colorhigh), (2,2))
        # trim by min/max color
        im, contours, hierarchy = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        mindis = 100000
        mincon = None
        rec = None
        for c in range(len(contours)):
          minRect = cv2.minAreaRect(contours[c])
          dist = math.sqrt((minRect[0][0] - point[0])**2 + (minRect[0][1]- point[1])**2)
          if dist < mindis:
            mindis = dist
            mincon = c
            rec = minRect

        box = cv2.boxPoints(rec)
        box = np.intp(box)
        mu = cv2.moments(contours[mincon])
        mc = (int(mu['m10'] / (mu['m00'] + 1e-5)), int(mu['m01'] / (mu['m00'] + 1e-5)))
        self.block_detections = mc
        self.block_contours = [box]
        minnorm = 1000
        colorName = "None"

        for c in self.colorBases:
              #print(np.linalg.norm(np.array(self.colorBases[c]) - avgc))
              if np.linalg.norm(np.array(self.colorBases[c] - avgc)) < minnorm:
                    minnorm = np.linalg.norm(np.array(self.colorBases[c] - avgc))
                    colorName = c
        
        self.color = colorName

    def detectBlocksInDepthImage(self):
        """!
        @brief      Detect blocks from depth

                    TODO: Implement a blob detector to find blocks in the depth image
        """

        pass

class ImageListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    self.camera = camera

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.VideoFrame = cv_image

class TagImageListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    self.camera = camera

  def callback(self,data):
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.TagImageFrame = cv_image

class TagDetectionListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.tag_sub = rospy.Subscriber(topic,AprilTagDetectionArray,self.callback)
    self.camera = camera
  def callback(self,data):
    self.camera.tag_detections = data
    for detection in data.detections:
      id = detection.id[0] - 1
      # get the position and orientation as numpy arrays
      position = detection.pose.pose.pose.position
      position = np.array([position.x, position.y, position.z])
      
      orientation = detection.pose.pose.pose.orientation
      orientation = np.array([orientation.x, orientation.y, orientation.z, orientation.w])
      

      # # calculate new depth based on depth camera
      # tagDetectionZ = position[2]      
      # # find pixel location
      # tag_pixel = (1/tagDetectionZ) * np.matmul(self.camera.intrinsic_matrix, position)
      # # from pixel location find depth
      # depthZ = float(self.camera.DepthFrameRaw[int(tag_pixel[1]), int(tag_pixel[0])]) * .001
      # position[2] = depthZ

      # add to list of detection pose/orientations 
      self.camera.ar_tag_detections[id].append(copy({'position': position, 'orientation': orientation}))
      if len(self.camera.ar_tag_detections[id]) > 100:
        garbage = self.camera.ar_tag_detections[id].pop(0)


class CameraInfoListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.tag_sub = rospy.Subscriber(topic,CameraInfo,self.callback)
    self.camera = camera
  def callback(self,data):
    self.camera.intrinsic_matrix = np.reshape(data.K, (3,3))
    self.camera.distortion = np.reshape(data.D, (5,1))

class DepthListener:
  def __init__(self, topic, camera):
    self.topic = topic
    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber(topic,Image,self.callback)
    self.camera = camera

  def callback(self,data):
    try:
      cv_depth = self.bridge.imgmsg_to_cv2(data, data.encoding)
    except CvBridgeError as e:
      print(e)
    self.camera.DepthFrameRaw += cv_depth
    self.camera.DepthFrameRaw = self.camera.DepthFrameRaw/2
    self.camera.ColorizeDepthFrame()

class VideoThread(QThread):
    updateFrame = pyqtSignal(QImage, QImage, QImage)

    def __init__(self, camera, parent=None):
        QThread.__init__(self, parent=parent) 
        self.camera = camera
        image_topic = "/camera/color/image_raw"
        depth_topic = "/camera/aligned_depth_to_color/image_raw"
        camera_info_topic = "/camera/color/camera_info"
        tag_image_topic = "/tag_detections_image"
        tag_detection_topic = "/tag_detections"
        image_listener = ImageListener(image_topic, self.camera)
        depth_listener = DepthListener(depth_topic, self.camera)
        tag_image_listener = TagImageListener(tag_image_topic, self.camera)
        camera_info_listener = CameraInfoListener(camera_info_topic, self.camera)
        tag_detection_listener = TagDetectionListener(tag_detection_topic, self.camera)


    def run(self):
        if __name__ == '__main__':
            cv2.namedWindow("Image window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Depth window", cv2.WINDOW_NORMAL)
            cv2.namedWindow("Tag window", cv2.WINDOW_NORMAL)
            time.sleep(0.5)
        while True:
            self.camera.processVideoFrame()
            rgb_frame = self.camera.convertQtVideoFrame()
            depth_frame = self.camera.convertQtDepthFrame()
            tag_frame = self.camera.convertQtTagImageFrame()
            if((rgb_frame != None)&(depth_frame != None)):
                self.updateFrame.emit(rgb_frame, depth_frame, tag_frame)
            time.sleep(0.03)
            if __name__ == '__main__':
                cv2.imshow("Image window", cv2.cvtColor(self.camera.VideoFrame,cv2.COLOR_RGB2BGR))
                cv2.imshow("Depth window", self.camera.DepthFrameRGB)
                cv2.imshow("Tag window", cv2.cvtColor(self.camera.TagImageFrame,cv2.COLOR_RGB2BGR))
                cv2.waitKey(3)
                time.sleep(0.03)

if __name__ == '__main__':
    camera = Camera()
    videoThread = VideoThread(camera)
    videoThread.start()
    rospy.init_node('realsense_viewer', anonymous=True)
    try:
      rospy.spin()
    except KeyboardInterrupt:
      print("Shutting down")
    cv2.destroyAllWindows()
