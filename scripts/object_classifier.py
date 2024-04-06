#!/usr/bin/env python
"""OpenCV feature detectors with ros Image Topics in python.

This example subscribes to a ros topic containing sensor_msgs 
Image. It converts the Image into a numpy.ndarray, 
then detects and marks features in that image. It finally displays 
and publishes the new image - again as Image topic.
"""
__author__ =  'Simon Haller <simon.haller at uibk.ac.at>'
__version__=  '0.1'
__license__ = 'BSD'
# Python libs
import sys, time

# numpy and scipy
import numpy as np
from scipy.ndimage import filters

# OpenCV
import cv2

# Ros libraries
import roslib
import rospy

# Ros Messages
from sensor_msgs.msg import Image
# We do not use cv_bridge it does not support Image in python
# from cv_bridge import CvBridge, CvBridgeError

VERBOSE=False

class ObjectDetection:

    def __init__(self):
        '''Initialize ros publisher, ros subscriber'''
        # topic where we publish
        self.image_pub = rospy.Publisher("kinect_classified",
            Image)
        # self.bridge = CvBridge()

        # subscribed Topic
        self.subscriber = rospy.Subscriber("kinect_rgb",
            Image, self.callback,  queue_size = 1)
        if VERBOSE :
            print "subscribed to kinect_rgb"

    def classify_objects(self, image):
        '''
        Use a YOLO model to classify the items in the frame. Return their bounding boxes and classes
        '''
        pass

    def callback(self, ros_data):
        '''Callback function of subscribed topic. 
        Here images get converted and features detected'''
        if VERBOSE :
            print 'received image of type: "%s"' % ros_data.format

        #### direct conversion to CV2 ####
        np_arr = np.fromstring(ros_data.data, np.uint8)
        # image_np = cv2.imdecode(np_arr, cv2.CV_LOAD_IMAGE_COLOR)
        image_np = cv2.imdecode(np_arr, cv2.IMREAD_COLOR) # OpenCV >= 3.0:
        image_mask = np.zeros([1920, 1080, 4]) ## Populate these with the bounding boxes of the classified items

        ## Use an object clasifier to get the bouding boxes & class of the objects
        # bboxes, classes = classify_objects(image_np)

        ## For each of the objects, create a rectangle on the black mask, change its color according to class
        cv2.draw_rectangle(image_mask, (100, 100), (400, 400), (255, 0,0, 0), -1)

        #### Create Image ####
        msg = Image()
        msg.header.stamp = rospy.Time.now()
        msg.format = "bgra8"
        msg.data = np.array(image_mask).tostring()
        # Publish new image
        self.image_pub.publish(msg)
        
        #self.subscriber.unregister()

def main(args):
    '''Initializes and cleanup ros node'''
    OD = ObjectDetection()
    rospy.init_node('ObjectDetection', anonymous=True)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print "Shutting down ROS Image feature detector module"
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)