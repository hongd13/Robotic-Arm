import numpy as np
import cv2


def display_image(image, name, scale=2, wait=False):
    """ 
    function to display an image 
    :param image: ndarray, the image to display
    :param name: string, a name for the window
    :param scale: int, optional, scaling factor for the image
    :param wait: bool, optional, if True, will wait for click/button to close window
    """
    cv2.namedWindow(name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(name, image.shape[1]*scale, image.shape[0]*scale)
    cv2.imshow(name, image)
    cv2.waitKey(0 if wait else 1)

def rgb_to_hsv(rgb):
    """
    Converts RGB to CV2 HSV values
    
    params:
    rgb - rgb array
    
    return:
    none
    """
    ms_paint_rgb = np.array(rgb, dtype=np.unit8)
    ms_paint_bgr = ms_paint_rgb[::-1]
    
    ms_paint_bgr = np.array([[ms_paint_bgr]], dtype=np.unit8)
    ms_paint_hsv = cv2.cvtColor(ms_paint_bgr, cv2.COLOR_BGR2HSV)
            
    print("HSV:", ms_paint_hsv)

red = np.array([
        [7, 63, 82],
        [3, 61, 77],
        [9, 49, 73],
        [17, 51, 76],
        [17, 49, 74],
        [18, 49, 76],
        [2, 67, 76],
        [6, 63, 78],
        [4, 50, 58],
        [16, 59, 82]
], np.cfloat)
red_noise = np.array([
        [17, 55, 58],
        [18, 52, 59],
        [18, 53, 60],
        [16, 51, 59],
        [17, 51, 58],
        [17, 50, 61],
        [16, 48, 65],
        [17, 53, 58],
        [16, 53, 58],
        [16, 51, 60],
        [17, 51, 58],
        [17, 51, 59],
        [16, 50, 58],
        [17, 50, 57],
        [15, 48, 58],
        [17, 50, 61],
        [17, 51, 61],
        [16, 52, 58],
        [16.8, 48.8, 65.9],
        [17.3, 55.3, 58.8]
], np.cfloat)
yellow = np.array([
        [52, 51, 80],
        [53, 61, 74],
        [50, 50, 70],
        [49, 43, 75],
        [51, 52, 70],
        [49, 49, 67],
        [49, 55, 67],
        [49, 49, 67],
        [49, 50, 69],
        [52, 57, 63],
        [50, 54, 60],
        [47, 40, 67],
        [53, 61, 78],
        [48, 43, 76],
        [52, 57, 78],
        [54, 57, 78],
        [54, 59, 81],
        [55, 56, 79],
        [55, 56, 79],
        [54, 59, 81],
        [54, 58, 74],
        [50, 52, 69],
        [48, 43, 76],
        [51, 52, 74],
        [54, 58, 77],
        [43, 37, 62],
        [49, 33, 71],
        [47, 31, 72],
        [41, 36, 70],
        [43, 41, 60],
        [45, 41, 56],
        [41, 36, 70],
        [45, 36, 60],
        [47, 40, 57],
        [42, 35, 61],
        [57, 53, 82],
        [52, 62, 69],
        [48, 35, 72],
        [46, 49, 59],
        [48, 52, 58]
], np.cfloat)

brown = np.array([
        [24, 26, 42],
        [30, 30, 44],
        [28, 32, 57],
        [35, 32, 58],
        [32, 31, 56],
        [32, 27, 66],
        [25, 21, 56],
        [28, 29, 59],
        [40, 38, 71],
        [37, 29, 60],
        [37, 29, 60],
        [16, 24, 46],
        [22, 24, 51],
        [17, 22, 42],
        [38, 36, 56],
        [26, 26, 47],
        [15, 20, 38],
        [20, 21, 43],
        [22, 24, 54],
        [31, 30, 52]
], np.cfloat)

brown_noise = np.array([
        [19.4, 37.4, 71.4],
        [18.2, 36.1, 71.8],
        [18.5, 37.2, 71.8],
        [16.5, 38.1, 71.0],
        [17.3, 36.7, 70.6],
        [17.3, 37.1, 69.8],
        [17.6, 38.6, 69.0],
        [17.9, 38.7, 67.8],
        [18.3, 38.3, 70.6],
        [19.1, 35.9, 72.2],
        [18.0, 39.3, 69.8],
        [18.3, 37.5, 72.2],
        [17.8, 35.0, 71.8],
        [17.3, 39.1, 66.3],
        [18.2, 38.4, 67.5],
        [18.2, 37.7, 68.6],
        [18.3, 38.8, 69.8],
        [16.4, 30.2, 71.4]
], np.cfloat)


def gimp_hsv_to_cv2(col):
    """
    Converts list of gimp hsv values to workable lists
    
    params:
    col - gimp colour hsv
    """
    
    col[:, 0] *= 0.5
    col[:, 1] *= 2.55
    col[:, 2] *= 2.55
    
    col = np.round(col, decimals=2)
    
    hsv_lower = [np.min(col[:, 0]), np.min(col[:, 1]), np.min(col[:, 2])]
    hsv_upper = [np.max(col[:, 0]), np.max(col[:, 1]), np.max(col[:, 2])]
    
    print("Lower:", hsv_lower)
    print("Upper:", hsv_upper)
    