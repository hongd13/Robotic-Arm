import numpy as np
import cv2
import math

from util import display_image



class Detector():
    
    def __init__(self):
        # centroid of image
        origin = [64, 32]
        
        # red hsv range, 0 lower 1 upper 
        self.red_hsv = [
            np.array([1, 124, 147], np.uint8),
            np.array([9, 174, 212], np.uint8)
        ]
        # red hsv noises
        self.red_noise = [
            np.array([7, 122, 145], np.uint8),
            np.array([9, 142, 169], np.uint8)
        ]
        
        # yellow hsv range, 0 lower 1 upper
        self.yellow_hsv = [
            np.array([20, 79, 142], np.uint8),
            np.array([28, 158, 209], np.uint8)
        ]
        
        # brown and hsv range, 0 lower 1 upper
        self.brown_hsv = [
            np.array([7, 51, 96], np.uint8),
            np.array([20, 100, 184], np.uint8)
        ]
        # brown noises, 0 lower 1 upper
        self.brown_noise = [
            np.array([8, 77, 169], np.uint8),
            np.array([9, 100, 184], np.uint8)
        ]
        
        # hd red
        self.hd_red_hsv = [
            np.array([178, 197, 207], np.uint8),
            np.array([180, 212, 223], np.uint8)
        ]
        # hd brown
        self.hd_brown_hsv = [
            np.array([10, 47, 112], np.uint8),
            np.array([22, 104, 194], np.uint8)
        ]
        # hd yellow
        self.hd_yellow_hsv = [
            np.array([28, 149, 210], np.uint8),
            np.array([30, 210, 225], np.uint8)
        ]
        
    
    def holding_cube(self, img, origin):
        a, box, b = self.get_holding_cube(img, origin)
        x1, y1, x2, y2 = box
        size = (abs(x1-x2) * abs(y1-y2))
        print(box)
        print("Check holding size: ", size)
        return size > 400
    
    def cube_aligned(self, box, centroid, origin):
        """
        Check whether the given cube in right under the gripper or not
        
        box - x1y1x2y2
        centroid - of cube
        origin - of camera
        
        returns:
        bool - True - aligned
                False - not aligned
        """
        if box == False:
            return False
            
        x1, y1, x2, y2 = box
        box_size = abs(x1-x2) * abs(y1-y2)

        if box_size >= 280 and box_size <= 309 and self.can_pick_up(centroid, origin): # 529 289
            return True
        return False
    
    def can_pick_up(self, cube, origin):
        """
        Cube is ready for pick up if the camera is 
        right above the cube's centroid
        """
        xc, yc = cube
        xo, yo = origin
        yo += 13 # origin offset, 13 = centre, 10 = edge
        xo -= 0.5
        if abs(xc - xo) <= 0.5 and abs(yc - yo) <= 0.5:
            return True
        else:
            return False
        
    def get_holding_cube(self, img, origin):
        """
        Get the closest cube, if it is big enough, it means it is being held,
        therefore return true, else return false
        
        params:
        img - raw camera input
        origin - origin
        
        returns:
        4 x False - no cubes in sight - failure
        best - x1, y1, x2, y2 of the closest cube to the camera's pov
        bool - True = cluster detected
                False = No cluster
        """
        
        #boxes, bb, reduced = self.get_BB(img, [self.brown_hsv], [self.brown_noise])
        
        # find and draw objects consisting of red, yellow and brown
        boxes, bb, reduced = self.get_BB(img, [self.red_hsv, self.yellow_hsv, self.brown_hsv, self.hd_red_hsv, self.hd_brown_hsv, self.hd_yellow_hsv], 
                                                [self.red_noise, self.brown_noise])
        # draw origin onto bb
        bb = cv2.rectangle(bb.copy(), (origin[0], origin[1]), (origin[0]+1, origin[1]+1), (0, 215, 255), 1)
        
        if len(boxes) == 0:
            return [False, False], False, False
        
        # current best cube - x, y, distance
        cur = [0, 0, -1]
        # best cube's corners
        best = [0, 0, 0, 0]
        if len(boxes) != 0:
            # if some cubes are in sight
            for box in boxes:
                x1, y1, x2, y2 = box
                
                new_dist = self.get_dist(box, origin)
                #print("new dist ", new_dist)
                if new_dist is not None and (cur[2] == -1 or new_dist[2] < cur[2]):
                    cur = new_dist
                    best = box
                    
        # draw the closest cube in red
        x1, y1, x2, y2 = best
        
        bb = cv2.rectangle(bb.copy(), (x1, y1), (x2, y2), (0, 0, 255), 1)
        display_image(reduced, 'reduced')
        display_image(bb, 'bb', wait=False)
        box_size = abs(x1-x2) * abs(y1-y2)
        return [cur[0], cur[1]], best, (box_size >= 620)
        
    def get_closest_cube(self, img, origin):
        """
        Find the relative position of the closest cube of the right size
        bigger than 50, smaller than 600
        If closest cube being > 600, it is probably a cluster, break it up!
        
        params:
        img - raw camera input
        
        returns:
        4 x False - no cubes in sight - failure
        best - x1, y1, x2, y2 of the closest cube to the camera's pov
        bool - True = cluster detected
                False = No cluster
        """
        
        #boxes, bb, reduced = self.get_BB(img, [self.brown_hsv], [self.brown_noise])
        
        # find and draw objects consisting of red, yellow and brown
        boxes, bb, reduced = self.get_BB(img, [self.red_hsv, self.yellow_hsv, self.brown_hsv], 
                                                [self.red_noise, self.brown_noise])
        # draw origin onto bb
        bb = cv2.rectangle(bb.copy(), (origin[0], origin[1]), (origin[0]+1, origin[1]+1), (0, 215, 255), 1)
        
        if len(boxes) == 0:
            return [False, False], False, False
        
        # current best cube - x, y, distance
        cur = [0, 0, -1]
        # best cube's corners
        best = [0, 0, 0, 0]
        if len(boxes) != 0:
            # if some cubes are in sight
            for box in boxes:
                x1, y1, x2, y2 = box
                
                new_dist = self.get_dist(box, origin)
                #print("new dist ", new_dist)
                if new_dist is not None and (cur[2] == -1 or new_dist[2] < cur[2]):
                    cur = new_dist
                    best = box
                    
        # draw the closest cube in red
        x1, y1, x2, y2 = best
        
        bb = cv2.rectangle(bb.copy(), (x1, y1), (x2, y2), (0, 0, 255), 1)
        display_image(reduced, 'reduced')
        display_image(bb, 'bb', wait=False)
        box_size = abs(x1-x2) * abs(y1-y2)
        return [cur[0], cur[1]], best, (box_size >= 620)
        
    def get_BB(self, image, hsv_s, hsv_null):
        """
        This method finds the bounding box positions for the specified hsv ranges, and draws them onto the image
        (reused code)
        
        params:
        image - the main image
        hsv_s - array of hsv ranges
        hsv_null - crate's hsv to be nullified
        
        returns:
        max - array of bounding boxes cooresponding to the specified colour range
        bb - image with bounding box drawn
        reduced_map - binary image
        """
        
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        binary_image = np.zeros(image.shape)
        
        if len(hsv_s) != 0:
            for col in hsv_s:
                mask = cv2.inRange(hsv, col[0], col[1]).astype(bool)
                binary_image[mask] = 1.0
        
        if len(hsv_null) != 0:
            for col in hsv_null:
                mask = cv2.inRange(hsv, col[0], col[1]).astype(bool)
                binary_image[mask] = 0
        
        binary_image = binary_image[:, :, 0]
        
        reduce_noise = cv2.morphologyEx(binary_image, cv2.MORPH_CLOSE, (3, 3))
        
        # find bounding box
        reduce_noise_uint8 = (reduce_noise * 255).astype(np.uint8)
        contour_img = np.zeros_like(reduce_noise_uint8)
        # find contours
        bb = image
        ret, thresh = cv2.threshold(reduce_noise_uint8, 10, 255, cv2.THRESH_BINARY)
        contours, hierarchy = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        
        boxes = []
        for cnt in contours:
            x, y, w, h = cv2.boundingRect(cnt)
            
            #epsilon = 0.15 * cv2.arcLength(cnt, True)
            #approx = cv2.approxPolyDP(cnt, epsilon, True)
            
            #x, y, w, h = cv2.boundingRect(approx)
            
            if w*h >= 70:
                # add only boxes bigger than 70
                boxes.append([x, y, x + w, y + h]) # add boxes to list
        
        # perform non-maximum suppression to organise overlapping bounding boxes
        boxes = np.array(boxes)

        confidences = np.ones(len(boxes))
        indices = cv2.dnn.NMSBoxes(boxes.tolist(), confidences, 0.5, 0.5)
        
        # compile the indices
        max = []
        for i in indices:
            x1, y1, x2, y2 = boxes[i]
            bb = cv2.rectangle(bb.copy(), (x1-1, y1-1), (x2-1, y2-1), (0, 255, 0), 1)
            max.append([x1-1, y1-1, x2-1, y2-1]) # get the boxes
        
        return max, bb, reduce_noise
        
    def get_dist(self, box, origin):
        """
        This function returns the Euclidean distance of the given bounding box to the origin
        
        params:
        box - x1, y1, x2, y2
        
        returns:
        pack - package of info
               centroid x, y
               distance to origin
        """
        x1, y1, x2, y2 = box
        centroid = [((x1+x2) / 2), ((y1+y2) / 2)]
        distance = math.sqrt((centroid[0] - origin[0])**2 + (centroid[1] - origin[1])**2)
        
        pack = centroid + [distance]
        
        return pack
        
