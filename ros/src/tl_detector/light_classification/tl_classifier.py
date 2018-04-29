from styx_msgs.msg import TrafficLight
import numpy as np
import cv2

class TLClassifier(object):
    def __init__(self):
        #TODO load classifier
        pass

    def get_classification(self, image):
        """Determines the color of the traffic light in the image
        Args:
            image (cv::Mat): image containing the traffic light
        Returns:
            int: ID of traffic light color (specified in styx_msgs/TrafficLight)
        """
     
        hsv_img = cv2.cvtColor(image,cv2.COLOR_BGR2HSV)
       
        # TODO The red color and the other colors are clearly visible 
        # instead of a AI based classifier simple color detection is used

        r1_min = np.array([0, 100, 100],np.uint8)
        r2_min = np.array([160, 100, 100],np.uint8)

        r1_max = np.array([10, 255, 255],np.uint8)        
        r2_max = np.array([179, 255, 255],np.uint8)

        frame_threshed1 = cv2.inRange(hsv_img, r1_min, r1_max) 
        frame_threshed2 = cv2.inRange(hsv_img, r2_min, r2_max) 
        if cv2.countNonZero(frame_threshed1) + cv2.countNonZero(frame_threshed2) > 50:
           return TrafficLight.RED

        y_min = np.array([40.0/360*255, 100, 100],np.uint8)
        y_max = np.array([66.0/360*255, 255, 255],np.uint8)
        frame_threshed3 = cv2.inRange(hsv_img, y_min, y_max)
        if cv2.countNonZero(frame_threshed3) > 50:
           return TrafficLight.YELLOW

        g_min = np.array([90.0/360*255, 100, 100],np.uint8)
        g_max = np.array([140.0/360*255, 255, 255],np.uint8)
        frame_threshed4 = cv2.inRange(hsv_img, g_min, g_max)
        if cv2.countNonZero(frame_threshed4) > 50:
           return TrafficLight.GREEN

        return TrafficLight.UNKNOWN
