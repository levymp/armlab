import time
import math
import numpy
from copy import copy
from copy import deepcopy
class comp(object):
    def __init__(self, rxarm, camera, sm, comp):
        self.rxarm = rxarm
        self.camera = camera
        self.sm = sm
        if comp == 1:
            self.comp1()
        elif comp == 2:
            self.comp2()




    def detectblockswrapper(self, num):
        self.rxarm.sleep()
        time.sleep(4)
        for i in range(num):
            self.camera.detectAll()
        time.sleep(4)
        self.rxarm.initialize()
        time.sleep(4)

    def comp1(self):
        if not self.camera.cameraCalibrated or not self.rxarm.initialized:
            print('CAMERA/ARM NOT INITIALIZED!')
            return
        # get all colors needed for competition
        colors = self.camera.colorBases.keys()
        self.current_state = 'execute'
        # get all block states
        # self.detectblockswrapper(10)
        self.rxarm.sleep()
        time.sleep(5)
        for j in range(10):
            self.camera.detectAll()
        self.rxarm.initialize()
        time.sleep(5)
        # go through all colors in block states and mark them as incomplete
        # for color in self.camera.blockState.keys():
        #     self.camera.blockState[color]['complete'] = False
        
        # check if blocks are at valid initial state
        # initialized = True
        
        # go through colors in comp and make sure we can see them
        # for color in colors:
        #     if self.camera.blockState[color]['pixel'][0] == 0:
        #         # raise error
        #         print('{color} WAS NOT FOUND! EXITING'.format(color=color))
        #         return
        #     if self.camera.blockState[color]['pixel'][1] > 0:
        #         initialized = False
        
        # check if the robot is not initalized
        # if not initialized:
        #     print('BLOCKS ARE NOT ON BOARD/CORRECT SIDE')

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
                # self.detectblockswrapper(10)
                for j in range(5):
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
            time.sleep(14)
            # time.sleep(self.rxarm.get_total_time()+1)
            
            # go to initialize state at the end
        self.sm.next_state = 'initialize_rxarm'




    def comp2(self):
        print("Starting Competition 2")
        print("Sleeping arm and detecting blocks")
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
                        time.sleep(8)
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

    def artist(self, draw):
        initial_point = [230, 230] # TODO: change point
        initial_position = self.pixel2coordinates(initial_point)
        positions = []
        if draw == 'circle':
            pass
            """
            radius = 0.01
            num_of_points = 16

            for theta in numpy.arange(0.0, 2.0 * math.pi, math.pi / num_of_points):
                x = radius * math.cos(theta)
                y = radius * math.sin(theta)
                #print(math.sqrt(math.pow(x, 2) + math.pow(y, 2)))
                position.append(initial_position[0] + x, initial_position[1] + y, initial_position[2])
            """
            
        elif draw == 'square':
            point_per_side = 4

            for i in range(-1,1):
                for j in range(-1,1):
                    if i == 0 and j == 0:
                        continue
                    x = i * .01
                    y = i * 10

                    position.append(initial_position[0] + x, initial_position[1] + y, initial_position[2])
            
        elif draw == 'elephant':
            pass
        else:
            pass



    def pixel2coordinates(self, point): 
        # take in a pixel near a block and get the block world frame coordiantes back 
        # this can then be fed into PICK/PLACE
        self.camera.last_click = (point[0], point[1])
        self.camera.blockDetector()
        pickCords = self.camera.pointToWorld(self.camera.block_detections)
        return pickCords


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


    def color2coordinates(self, color, color_vec, initial_coordinates, offset = 0.075, axis='x'):
        count = -1
        for color_key in color_vec:
            count += 1
            if color_key == color:
                break
        print("INIT COORDINATES", initial_coordinates)
        print("COLOR", color)
        print("COUNT", count)
        new_coordinates = deepcopy(initial_coordinates)
        if axis=='x':
            new_coordinates[0] += offset*count
        elif axis == 'y':
            new_coordinates[1] += offset*count
        elif axis == 'z':
            new_coordinates[2] += offset*count
        
        return new_coordinates
