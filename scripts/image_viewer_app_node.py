#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

# Sample NEPI Process Script.
# 1. Converts ROS pointcloud2 to Open3d pointcloud
# 2. Blank area for custom code
# 3. Converts Open3d pointcloud back to ROS pointcloud2


import os
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy
import time
import sys
import numpy as np
import time


from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_app_image_viewer.msg import ImageSelection

from nepi_edge_sdk_base import nepi_ros

from nepi_edge_sdk_base.save_data_if import SaveDataIF
from nepi_edge_sdk_base.save_cfg_if import SaveCfgIF


#########################################

# Factory Control Values

#########################################
# Node Class
#########################################

UPDATE_IMAGE_SUBS_RATE_HZ = 1
UPDATE_SAVE_DATA_CHECK_RATE_HZ = 10

class NepiImageViewerApp(object):

  FACTORY_SELECTED_TOPICS = ["None","None","None","None"]


  update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
  update_save_data_check_interval_sec = float(1)/UPDATE_SAVE_DATA_CHECK_RATE_HZ

  data_products = ["image_0","image_1","image_2","image_3"]
  img_subs_dict = dict()
  ###################
  ## App Callbacks

  def resetAppCb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~iv_app/selected_topics', self.FACTORY_SELECTED_TOPICS)
    self.publish_status()

  def setImageTopicCb(self,msg):
    #rospy.loginfo(msg)
    img_index = msg.image_index
    img_topic = msg.image_topic
    found_topic = nepi_ros.find_topic(img_topic)
    if img_index > -1 and img_index < 4 and found_topic != "":
      current_sel = rospy.get_param('~iv_app/selected_topics', self.init_selected_topics)
      current_sel[img_index] = found_topic
    rospy.set_param('~iv_app/selected_topics', current_sel)
    self.publish_status()


  #######################
  ### Node Initialization
  def __init__(self):
    node_name = "image_viewer_app"
    rospy.init_node(name=node_name)


    rospy.loginfo("IMGVIEW_APP: : Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)
   

    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_product_names = self.data_products)
    # Temp Fix until added as NEPI ROS Node
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer)


    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetAppCb, queue_size = 10)
    self.initParamServerValues(do_updates=False)

    ## App Subscribers ########################################################
    set_image_topic_sub = rospy.Subscriber("~set_image_topic", ImageSelection, self.setImageTopicCb, queue_size = 10)
  
    ## App Publishers
    self.sel_status_pub = rospy.Publisher("~status", String, queue_size=1, latch=True)

    # Give publishers time to setup
    time.sleep(1)

    # Publish Status
    self.publish_status()


    rospy.Timer(rospy.Duration(self.update_image_subs_interval_sec), self.updateImageSubsThread)
    ## Initiation Complete
    rospy.loginfo("resetAppCb:  Initialization Complete")

    #Set up node shutdown
    rospy.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    rospy.spin()


  #######################
  ### Config Functions

  def saveConfigCb(self, msg):  # Just update class init values. Saving done by Config IF system
    pass # Left empty for sim, Should update from param server

  def setCurrentAsDefault(self):
    pass # We only use the param server, no member variables to apply to param server

  def updateFromParamServer(self):
    # Don't need to run any additional functions
    pass

  def initParamServerValues(self,do_updates = True):
      self.init_selected_topics = rospy.get_param('~iv_app/selected_topics', self.FACTORY_SELECTED_TOPICS)
      self.resetParamServer(do_updates)

  def resetParamServer(self,do_updates = True):
      rospy.set_param('~iv_app/selected_topics', self.init_selected_topics)
      if do_updates:
          self.updateFromParamServer()
          self.publish_status()



  ###################
  ## Status Publishers
  def publish_status(self):
    sel_topics = rospy.get_param('~iv_app/selected_topics',self.init_selected_topics)
    status_msg = str(sel_topics)
    if not rospy.is_shutdown():
      self.sel_status_pub.publish(status_msg)


  #######################
  # Update Image Topic Subscribers Thread

  def updateImageSubsThread(self,timer):
    # Subscribe to topic image topics if not subscribed
    sel_topics = rospy.get_param('~iv_app/selected_topics',self.init_selected_topics)
    for sel_topic in sel_topics:
      if sel_topic != "" and sel_topic != "None" and sel_topic not in self.img_subs_dict.keys():
        if nepi_ros.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_img = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          rospy.loginfo("Subscribing to topic: " + sel_topic)
          rospy.loginfo("with topic_uid: " + topic_uid)
          img_sub = rospy.Subscriber(sel_topic, Image, lambda msg: self.imageCb(msg, sel_topic), queue_size = 10)
          self.img_subs_dict[sel_topic] = img_sub
          rospy.loginfo("IMG_VIEW_APP:  Image: " + sel_topic + " registered")
    # Unregister image subscribers if not in selected images list
    unreg_topic_list = []
    for topic in self.img_subs_dict.keys():
      if topic not in sel_topics:
          img_sub = self.img_subs_dict[topic]
          img_sub.unregister()
          rospy.loginfo("IMG_VIEW_APP: Image: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.img_subs_dict.pop(topic)
    

  def imageCb(self,msg,topic):
    pass
 
      
    
  #######################
  # Data Saving Funcitons
 

  def save_img2file(self,data_product,cv2_img,ros_timestamp):
      if self.save_data_if is not None:
          saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
          snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
          # Save data if enabled
          if saving_is_enabled or snapshot_enabled:
              if cv2_img is not None:
                  if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                      full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                              "image_viewer_app-" + data_product, 'png')
                      if os.path.isfile(full_path_filename) is False:
                          cv2.imwrite(full_path_filename, cv2_img)
                          self.save_data_if.data_product_snapshot_reset(data_product)

             

  #######################
  # Utility Funcitons   
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("IMG_VIEW_APP:  Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiImageViewerApp()





