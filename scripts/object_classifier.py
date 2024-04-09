#!/usr/bin/env python
from __future__ import print_function

import roslib
import sys
import rospy
import cv2 
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np

def inliers_indices(data, m = 1.5):
    q75, q25 = np.percentile(data, [75, 25])
    median = np.median(data)
    iqr = q75 - q25
    return np.logical_and(data > (median - m * iqr), data < (median + m * iqr))

class Matcher:
    def __init__(self, paths: list, object_types):
        FLANN_INDEX_KDTREE = 3
        index_params = dict(algorithm = FLANN_INDEX_KDTREE, trees = 5)
        search_params = dict(checks=200) # or pass empty dictionary
        self.sift = cv2.SIFT_create()
        self.flann = cv2.FlannBasedMatcher(index_params,search_params)
        self.object_descriptors = []
        self.object_types = object_types

        for path in paths:
            img = cv2.imread(path, cv2.IMREAD_GRAYSCALE)
            kp, des = self.sift.detectAndCompute(img,None)
            self.object_descriptors.append(des)

    def match(self, img_in_context):
            kp, des2 = self.sift.detectAndCompute(img_in_context,None) # query
            img_mask = np.zeros([1080, 1920, 4], np.uint8)

            for j, obj_des in enumerate(self.object_descriptors):
                matches = self.flann.knnMatch(obj_des, des2, k=2)

                # Get the points of the good matches in the query image
                good_matches_query = []
                # ratio test as per Lowe's paper
                for _, (m,n) in enumerate(matches):
                    if m.distance < 0.7*n.distance: # higher the value the more they let through
                        good_matches_query.append(m)
                
                if len(good_matches_query) < 20:
                    continue

                pts = []

                col = (0, 0, 0, 0)
                if self.object_types[j] == "box":
                    col = (255, 0, 0, 0)
                elif self.object_types[j] == "cup":
                    col = (0, 255, 0, 0)   
                elif self.object_types[j] == "fries":
                    col = (0, 0, 255, 0)   

                for match in good_matches_query:
                    pt = kp[match.trainIdx].pt
                    
                    pts.append(pt)
            
                pts_x = np.array(pts)[:,0]
                x_inliers = inliers_indices(pts_x, 1.5)
                # pts_y = np.array(pts)[:,1]
                # y_inliers = inliers_indices(pts_y)

                x_min = np.min(pts_x[x_inliers])
                x_max = np.max(pts_x[x_inliers])

                # Make a rectangle the entire height of the image
                tl = (x_min,0)
                br = (x_max,img_in_context.shape[0])

                cv2.rectangle(img_mask, np.int32(tl), np.int32(br), col, -1, )

            return img_mask
            
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("kinect_classified",Image, queue_size=1)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("kinect_rgb",Image,self.callback, queue_size=1)

    self.project_id = "10k"
    self.model_version = 4

    paths = ["/home/jetson/catkin_ws/src/kinect_publisher/scripts/box_3.jpg", "/home/jetson/catkin_ws/src/kinect_publisher/scripts/fries_2.jpg", "/home/jetson/catkin_ws/src/kinect_publisher/scripts/cup_3.jpg"]
    objects = ["box", "cup", "fries"]
    self.matcher = Matcher(paths, objects)
  
  
  def callback(self,data):
    cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
    rospy.loginfo("cv_image_shape {}".format(cv_image.shape))
    success = 0
    img = cv2.imread("/home/jetson/catkin_ws/src/kinect_publisher/scripts/query_2.jpg", cv2.IMREAD_GRAYSCALE)
    try:
      cv_image = self.bridge.imgmsg_to_cv2(data, "bgra8")
      gray = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2GRAY)
      image_mask = self.matcher.match(gray)
      # image_mask = self.matcher.match(img)

      # self.image_pub.publish(self.bridge.cv2_to_imgmsg(cv_image, "bgra8"))
      rospy.loginfo("Publishing mask size {}".format(image_mask.shape))
      img_msg = self.bridge.cv2_to_imgmsg(image_mask, encoding="bgra8")
      img_msg.header = data.header
      self.image_pub.publish(img_msg)
      success = 1
    except CvBridgeError as e:
      print(e)

    finally:

      if not success:
        image_blank = np.zeros([1080, 1920, 4], np.uint8) ## Populate these with the bounding boxes of the classified items

        rospy.loginfo("Publishing blank size {}".format(image_blank.shape))
        img_msg = self.bridge.cv2_to_imgmsg(image_blank, encoding="bgra8")
        img_msg.header = data.header
        self.image_pub.publish(img_msg)

      success = 0
        

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)