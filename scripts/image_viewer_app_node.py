#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus <https://www.numurus.com>.
#
# This file is part of nepi applications (nepi_apps) repo
# (see https://https://github.com/nepi-engine/nepi_apps)
#
# License: nepi applications are licensed under the "Numurus Software License", 
# which can be found at: <https://numurus.com/wp-content/uploads/Numurus-Software-License-Terms.pdf>
#
# Redistributions in source code must retain this top-level comment block.
# Plagiarizing this software to sidestep the license obligations is illegal.
#
# Contact Information:
# ====================
# - mailto:nepi@numurus.com
#


import os
# ROS namespace setup
#NEPI_BASE_NAMESPACE = '/nepi/s2x/'
#os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]

import time
import sys
import numpy as np
import time
import subprocess
import threading


from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_app_image_viewer.msg import ImageSelection
from nepi_ros_interfaces.msg import StringArray

from nepi_sdk import nepi_ros
from nepi_sdk import nepi_utils


from nepi_api.node_if import NodeClassIF
from nepi_api.messages_if import MsgIF
from nepi_api.system_if import SaveDataIF
from nepi_api.system_if import SaveCfgIF


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

  data_products = ["image0","image1","image2","image3"]
  img_subs_dict = dict()


  #######################
  ### Node Initialization



  DEFAULT_NODE_NAME = "app_image_viewer" # Can be overwitten by luanch command
  def __init__(self):
    #### APP NODE INIT SETUP ####
    nepi_ros.init_node(name= self.DEFAULT_NODE_NAME)
    self.class_name = type(self).__name__
    self.base_namespace = nepi_ros.get_base_namespace()
    self.node_name = nepi_ros.get_node_name()
    self.node_namespace = nepi_ros.get_node_namespace()

    ##############################  
    # Create Msg Class
    self.msg_if = MsgIF(log_name = self.class_name)
    self.msg_if.pub_info("Starting IF Initialization Processes")

    ##############################     
    # Initialize Class Variables


    ##############################
    ### Setup Node

    # Configs Config Dict ####################
    self.CFGS_DICT = {
            'init_callback': self.initCb,
            'reset_callback': self.resetCb,
            'factory_reset_callback': self.factoryResetCb,
            'init_configs': True,
            'namespace': self.node_namespace
    }

    # Params Config Dict ####################
    self.PARAMS_DICT = {
        'selected_topics': {
            'namespace': self.node_namespace,
            'factory_val': []
        }
    }

    # Publishers Config Dict ####################
    self.PUBS_DICT = {
        'status_pub': {
            'namespace': self.node_namespace,
            'topic': 'status',
            'msg': StringArray,
            'qsize': 1,
            'latch': True
        }
    }

    # Subscribers Config Dict ####################
    self.SUBS_DICT = {
        'set_topic': {
            'namespace': self.node_namespace,
            'topic': 'set_topic',
            'msg': ImageSelection,
            'qsize': 10,
            'callback': self.setImageTopicCb, 
            'callback_args': ()
        }
    }


    # Create Node Class ####################
    self.node_if = NodeClassIF(
                    configs_dict = self.CFGS_DICT,
                    params_dict = self.PARAMS_DICT,
                    pubs_dict = self.PUBS_DICT,
                    subs_dict = self.SUBS_DICT,
                    log_class_name = True
    )

    ready = self.node_if.wait_for_ready()



    ##############################
    self.initCb(do_updates = True)
    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_product_names = self.data_products)


    # Publish Status
    self.publish_status()

    time.sleep(1)
    self.nepi_ros.start_timer_process(0.5, self.statusPublishCb)
    # Give publishers time to setup
    time.sleep(1)

    nepi_ros.timer(self.update_image_subs_interval_sec, self.updateImageSubsThread)
    ## Initiation Complete
    self.msg_if.pub_info("factoryResetCbCb:  Initialization Complete")

    #Set up node shutdown
    nepi_ros.on_shutdown(self.cleanup_actions)
    # Spin forever (until object is detected)
    nepi_ros.spin()







  ###################
  ## App Callbacks


  def setImageTopicCb(self,msg):
    #self.msg_if.pub_info(str(msg))
    img_index = msg.image_index
    img_topic = msg.image_topic
    current_sel = self.node_if.get_param('selected_topics')
    current_sel[img_index] = img_topic
    self.node_if.set_param('selected_topics', current_sel)
    self.publish_status()




  #######################
  ### Config Functions



  def initCb(self,do_updates = False):
      if do_updates == True:
        self.resetCb(do_updates)

  def resetCb(self,do_updates = True):
      if do_updates:
          self.publish_status()

  def factoryResetCb(self):
    self.publish_status()

  ###################
  ## Status Publishers

  def statusPublishCb(self,timer):
      self.publish_status()

  def publish_status(self):
    sel_topics = self.node_if.get_param('selected_topics')
    #for i, topic in enumerate(sel_topics):
      #if topic != "None":
        #if nepi_ros.find_topic(topic) == "":
          #sel_topics[i] = "None"
    status_msg = sel_topics
    if not nepi_ros.is_shutdown():
      self.node_if.publish_pub('status_pub',status_msg)


  #######################
  # Update Image Topic Subscribers Thread

  def updateImageSubsThread(self,timer):
    # Subscribe to topic image topics if not subscribed
    sel_topics = self.node_if.get_param('selected_topics')
    #self.msg_if.pub_warn("Selected images: " + str(sel_topics))
    #self.msg_if.pub_warn("Subs dict keys: " + str(self.img_subs_dict.keys()))
    for i, sel_topic in enumerate(sel_topics):
      if sel_topic != "" and sel_topic != "None" and sel_topic not in self.img_subs_dict.keys():
        if nepi_ros.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_img = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          self.msg_if.pub_info("Subscribing to topic: " + sel_topic)
          self.msg_if.pub_info("with topic_uid: " + topic_uid)
          data_product = "image" + str(i)
          img_sub = self.nepi_ros.create_subscriber(sel_topic, Image, lambda img_msg: self.imageCb(img_msg, data_product), queue_size = 10)
          self.img_subs_dict[sel_topic] = img_sub
          self.msg_if.pub_info("IMG_VIEW_APP:  Image: " + sel_topic + " registered")
    # Unregister image subscribers if not in selected images list
    unreg_topic_list = []
    for topic in self.img_subs_dict.keys():
      if topic not in sel_topics:
          img_sub = self.img_subs_dict[topic]
          img_sub.unregister()
          self.msg_if.pub_info("IMG_VIEW_APP: Image: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.img_subs_dict.pop(topic)
    

  def imageCb(self,img_msg,data_product):
    self.save_data_if.save_ros_img2file(data_product,img_msg,img_msg.header.stamp)


 
      


             

  #######################
  # Utility Funcitons   
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    self.msg_if.pub_info("IMG_VIEW_APP:  Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  NepiImageViewerApp()





