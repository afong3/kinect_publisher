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
from inference_sdk import InferenceHTTPClient
import message_filters

class Matcher:
    def __init__(self, paths: list, object_types):
      pass  

    def match(self, img_in_context):
      pass
            
class image_converter:

  def __init__(self):
    self.image_pub = rospy.Publisher("kinect_classified",Image, queue_size=1)
    self.depth_pub = rospy.Publisher("kinect_depth_classified",Image, queue_size=1)
    self.bridge = CvBridge()
    self.depth_sub = message_filters.Subscriber("kinect_depth", Image)
    self.image_sub = message_filters.Subscriber("kinect_rgb",Image)
    ts = message_filters.TimeSynchronizer([self.image_sub, self.depth_sub], 10)
    ts.registerCallback(self.callback)

    self.project_id = "mcdonalds-waste"
    self.model_version = 1
    self.client = InferenceHTTPClient(api_url="https://detect.roboflow.com", api_key="QPB2HQqf5FT2inukdAPH")
    
    # self.project_id = "10k"
    # self.model_version = 4  
  
  def callback(self,msg_color, msg_depth):
    cv_image = self.bridge.imgmsg_to_cv2(msg_color, "bgra8")
    rospy.loginfo("cv_image_shape {}".format(cv_image.shape))
    results = self.client.infer(cv_image, model_id=f"{self.project_id}/{self.model_version}")
    success = 0
    colors = {"compost": (255, 0, 0), "landfill": (0, 255, 0), "recycle": (0, 0, 255)}
    for detection in results["predictions"]:
      x, y, width, height = detection['x'], detection['y'], detection['width'], detection['height']
      confidence, class_name = detection['confidence'], detection['class']

      # Draw bounding box
      
      cv2.rectangle(cv_image, (int(x - width/2), int(y - height/2)), (int(x + width/2), int(y + height/2)), colors[class_name], -1)

      # Add label and confidence score
      label = f'{class_name}: {confidence:.2f}'


      rospy.loginfo("Class: {}\nConfidence: {}\n\n".format(class_name, confidence))
      success = 1

    if not success:
      rospy.loginfo("Found no items")
    try:
      img_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding="bgra8")
      img_msg.header = msg_color.header
      self.image_pub.publish(img_msg)
      self.depth_pub.publish(msg_depth)

    except CvBridgeError as e:
      print(e)

    finally:

      # if not success:
      #   image_blank = np.zeros([1080, 1920, 4], np.uint8) ## Populate these with the bounding boxes of the classified items

      #   rospy.loginfo("Publishing blank size {}".format(image_blank.shape))
      #   img_msg = self.bridge.cv2_to_imgmsg(image_blank, encoding="bgra8")
      #   img_msg.header = data.header
      #   self.image_pub.publish(img_msg)

      # success = 0
      reason = "All done here"
      rospy.signal_shutdown(reason)
        

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")

if __name__ == '__main__':
    main(sys.argv)