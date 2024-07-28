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


#ToDo
#- Add pointcloud render enable control to node params, add node subscriber, add to to RenderStatus msg, and have asher add control to RUI
#- Figure out why no pointcloud data is showing up in rendered image when process clipping is disabled. 
#- Create add_pointclouds function in nepi_img.py that supports combining bw and rgb pointclouds and replace in node Add process
#- Implement age_filter before pointcloud combine process
#- Apply transforms to each point cloud before combining
#- Add fit_pointclouds function to nepi_img.py and add to node comnbine options
#- Improve pointcloud pub latency. Break rendering into its own node that this node starts up.  The new node would subscribe to this noodes pointcloud topic 

import os
# ROS namespace setup
NEPI_BASE_NAMESPACE = '/nepi/s2x/'
os.environ["ROS_NAMESPACE"] = NEPI_BASE_NAMESPACE[0:-1]
import rospy


import time
import sys
import numpy as np
import time


from std_msgs.msg import UInt8, Empty, String, Bool, Float32, Int32
from sensor_msgs.msg import Image
from nepi_ros_interfaces.msg import ImageSelection

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

class image_viewer_app(object):

FACTORY_SELECTED_TOPICS = ["None","None","None","None"]


update_image_subs_interval_sec = float(1)/UPDATE_IMAGE_SUBS_RATE_HZ
update_save_data_check_interval_sec = float(1)/UPDATE_SAVE_DATA_CHECK_RATE_HZ

data_products = ["image_0","image_1","image_2","image_3"]

###################
## App Callbacks

  def resetApimgb(self,msg):
    self.resetApp()

  def resetApp(self):
    rospy.set_param('~iv_app/selected_topics', self.FACTORY_SELECTED_TOPICS)
    self.publish_status()

  def setImageTopicCb(self,msg):
    #rospy.loginfo(msg)
    img_index = msg.image_index
    img_topic = msg.image_topic
    found_topic = nepi_ros.find_topic(img_topic)
    if img_index > -1 and img index < 4 and found_topic != "":
      current_sel = rospy.get_param('~iv_app/selected_topics', self.init_selected_topics)
      current_sel[img_index] = found_topic
    rospy.set_param('~iv_app/selected_topics', current_sel)
    self.publish_status()


  #######################
  ### Node Initialization
  def __init__(self):
   
    rospy.loginfo("IV_APP: : Starting Initialization Processes")
    self.initParamServerValues(do_updates = False)
    self.resetParamServer(do_updates = False)
   

    # Set up save data and save config services ########################################################
    self.save_data_if = SaveDataIF(data_product_names = self.data_products)
    # Temp Fix until added as NEPI ROS Node
    thisNamespace = NEPI_BASE_NAMESPACE + "image_viewer_app"
    self.save_cfg_if = SaveCfgIF(updateParamsCallback=self.initParamServerValues, 
                                 paramsModifiedCallback=self.updateFromParamServer,
                                 namespace = thisNamespace)


    ## App Setup ########################################################
    app_reset_app_sub = rospy.Subscriber('~reset_app', Empty, self.resetApimgb, queue_size = 10)
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
    rospy.Timer(rospy.Duration(self.update_save_data_check_interval_sec), self.updateSaveDataCheckThread)
    ## Initiation Complete
    rospy.loginfo("IV_APP:  Initialization Complete")


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
    status_msg = ImageSlection()
    sel_topics = rospy.get_param('~iv_app/selected_topics',self.init_selected_topics)
    status_msg.data = str(sel_topics)
    if not rospy.is_shutdown():
      self.sel_status_pub.publish(status_msg)


  #######################
  # Update Image Topic Subscribers Thread

  def updateImageSubsThread(self,timer):
    # Subscribe to topic pointcloud topics if not subscribed
    sel_topics = rospy.get_param('~iv_app/selected_topics',self.init_selected_pointclouds)
    for sel_topic in sel_topics:
      if sel_topic != "" and sel_topic not in self.img_subs_dict.keys():
        if nepi_ros.check_for_topic(sel_topic):
          topic_uid = sel_topic.replace('/','')
          exec('self.' + topic_uid + '_img = None')
          exec('self.' + topic_uid + '_timestamp = None')
          exec('self.' + topic_uid + '_frame = None')
          exec('self.' + topic_uid + '_lock = threading.Lock()')
          rospy.loginfo("Subscribing to topic: " + sel_topic)
          rospy.loginfo("with topic_uid: " + topic_uid)
          img_sub = rospy.Subscriber(sel_topic, Image, lambda msg: self.pointcloudCb(msg, sel_topic), queue_size = 10)
          self.img_subs_dict[sel_topic] = img_sub
          rospy.loginfo("IV_APP:  Pointcloud: " + sel_topic + " registered")
    # Unregister pointcloud subscribers if not in selected pointclouds list
    unreg_topic_list = []
    for topic in self.img_subs_dict.keys():
      if topic not in sel_topics:
          img_sub = self.img_subs_dict[topic]
          img_sub.unregister()
          rospy.loginfo("IV_APP:  Pointcloud: " + topic + " unregistered")
          unreg_topic_list.append(topic) # Can't change dictionary while looping through dictionary
    for topic in unreg_topic_list: 
          self.img_subs_dict.pop(topic)
    # Update primary pointcloud if needed
    primary_img = rospy.get_param('~iv_app/primary_pointcloud', self.init_primary_pointcloud)
    #print(self.img_subs_dict.keys())
    if primary_img not in self.img_subs_dict.keys():
      if len(self.img_subs_dict.keys()) > 0:
        primary_img = list(self.img_subs_dict.keys())[0]
      else:
        primary_img = "None"
      if primary_img != "None":
        rospy.loginfo("IV_APP:  Primary pointcloud set to: " + primary_img)
    rospy.set_param('~iv_app/primary_pointcloud', primary_img)
    

  def pointcloudCb(self,msg,topic):
      if topic != "":
        topic_uid = topic.replace('/','')
        transforms_dict = rospy.get_param('~iv_app/transforms_dict',self.init_transforms_dict)
        if topic in transforms_dict.keys():
          transform = transforms_dict[topic]
        else:
          transform = ZERO_TRANSFORM
          transforms_dict[topic] = transform  
        if topic in self.img_subs_dict.keys():
          eval('self.' + topic_uid + '_lock').acquire()
          exec('self.' + topic_uid + '_timestamp = msg.header.stamp')
          exec('self.' + topic_uid + '_frame = msg.header.frame_id')
          o3d_img = nepi_img.rosimg_to_o3dimg(msg, remove_nans=False)
          # ToDo: Apply Frame Transforms before assigning and releasing
          exec('self.' + topic_uid + '_img = o3d_img')
          eval('self.' + topic_uid + '_lock').release()


  def updateDataProductsThread(self,timer):
    # Check if new data is needed

    img_has_subscribers = (self.proc_img_pub.get_num_connections() > 0)
    img_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud')
    img_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud')
    need_img = (img_has_subscribers is True) or (img_saving_is_enabled is True) or (img_snapshot_enabled is True)


    img_has_subscribers = (self.view_img_pub.get_num_connections() > 0)
    img_saving_is_enabled = self.save_data_if.data_product_saving_enabled('pointcloud_image')
    img_snapshot_enabled = self.save_data_if.data_product_snapshot_enabled('pointcloud_image')
    need_img = (img_has_subscribers is True) or (img_saving_is_enabled is True) or (img_snapshot_enabled is True)

    ros_frame_id = rospy.get_param('~iv_app/process/frame_3d', self.init_proc_frame_3d)
    primary_img = rospy.get_param('~iv_app/primary_pointcloud', self.init_primary_pointcloud)

    if (need_img or need_img and primary_img != "None"):
      o3d_img = None
      # Combine selected 
      age_filter_s = rospy.get_param('~iv_app/age_filter_s', self.init_age_filter_s)
      combine_option = rospy.get_param('~iv_app/combine_option',self.init_combine_option)
      current_time = rospy.get_rostime()
      img_add_count = 0

      # Get priamary pointcloud
      topic_puid = primary_img.replace('/','')
      if primary_img in self.img_subs_dict.keys():
        eval('self.' + topic_puid + '_lock').acquire()
        ros_timestamp = eval('self.' + topic_puid + '_timestamp')
        primary_img_frame = eval('self.' + topic_puid + '_frame')
        if ros_timestamp is not None:
          #img_age = current_time - ros_timestamp_add
          img_age = 0.0 # Temp 
          if img_age <= age_filter_s:
            o3d_pimg = eval('self.' + topic_puid + '_img')
            if o3d_pimg is not None:
              o3d_img = copy.deeimgopy(o3d_pimg)
              img_add_count += 1
        eval('self.' + topic_puid + '_lock').release()
        if ros_frame_id == "primary_img_frame":
          ros_frame_id = primary_img_frame

      if o3d_img is not None:
        # Add remaining selected pointclouds
        for topic in self.img_subs_dict.keys():
          topic_uid = topic.replace('/','')
          if topic_uid != topic_puid:  # Skip the primary pointcloud
            ros_timestamp_add = None
            if topic in self.img_subs_dict.keys():
              eval('self.' + topic_uid + '_lock').acquire()
              ros_timestamp_add = eval('self.' + topic_uid + '_timestamp')
              o3d_img_add = eval('self.' + topic_uid + '_img')
              eval('self.' + topic_uid + '_lock').release()
            #else:
              #rospy.loginfo("IV_APP:  Combine pointcloud not registered yet: " + topic_puid)
            if ros_timestamp_add is not None:
              #img_age = current_time - ros_timestamp_add
              img_age = 0.0 # Temp 
              if o3d_img_add is not None:
                if img_age <= age_filter_s:
                  if combine_option == 'Add':
                    o3d_img += o3d_img_add
                    img_add_count += 1
                  if ros_timestamp_add > ros_timestamp:
                    ros_timestamp = ros_timestamp_add
  
      if img_add_count > 0:
        self.img_min_range_m = nepi_img.get_min_range(o3d_img)
        self.img_max_range_m = nepi_img.get_max_range(o3d_img)

        if self.img_max_range_m != 0:
          # Process Combined Pointcloud
          clip_enable = rospy.get_param('~iv_app/process/clip_range_enabled', self.init_proc_clip_range_enabled )
          if clip_enable:
            min_m = rospy.get_param('~iv_app/process/range_min_m', self.init_proc_range_min_m)
            max_m = rospy.get_param('~iv_app/process/range_max_m', self.init_proc_range_max_m)
            o3d_img = nepi_img.range_clip(o3d_img, min_m, max_m)

          if self.bounding_box3d_topic != "NONE" and self.bounding_box3d_msg is not None:
            clip_box_msg = copy.deeimgopy(self.bounding_box3d_msg)
            center = clip_box_msg.box_center_m
            extent = clip_box_msg.box_extent_xyz_m
            rotation = clip_box_msg.box_rotation_rpy_deg
            o3d_img = nepi_img.clip_bounding_box(o3d_img, center, extent, rotation)



          k_points = rospy.get_param('~iv_app/process/uniform_downsample_k_points',self.init_proc_uniform_downsample_k_points)
          if k_points > 0:
            o3d_img = nepi_img.uniform_down_sampling(o3d_img, k_points)

          num_neighbors = rospy.get_param('~iv_app/process/outlier_removal_num_neighbors',self.init_proc_outlier_removal_num_neighbors)   
          if num_neighbors > 0:
            statistical_outlier_removal_std_ratio = 2.0
            [o3d_img, ind] = nepi_img.statistical_outlier_removal(o3d_img, num_neighbors, statistical_outlier_removal_std_ratio)

          voxel_size_m = rospy.get_param('~iv_app/process/voxel_downsample_size',self.init_proc_voxel_downsample_size)
          if voxel_size_m > 0:
            o3d_img = nepi_img.voxel_down_sampling(o3d_img, voxel_size_m)

          # Publish and Save Pointcloud Data
          if img_has_subscribers:
            # ToDo Convert to map frame if selected
            ros_img_out_msg = nepi_img.o3dimg_to_rosimg(o3d_img, stamp=ros_timestamp, frame_id=ros_frame_id)
            if not rospy.is_shutdown():
              self.proc_img_pub.publish(ros_img_out_msg)

          if img_saving_is_enabled is True or img_snapshot_enabled is True:
            self.save_img2file('pointcloud',o3d_img,ros_timestamp)


          if need_img:
            # Render the pointcloud image
            img_width = rospy.get_param('~iv_app/render/image_width',  self.init_image_width)
            img_height = rospy.get_param('~iv_app/render/image_height', self.init_image_height)
            start_range_ratio = rospy.get_param('~iv_app/render/start_range_ratio', self.init_view_start_range_ratio)
            stop_range_ratio = rospy.get_param('~iv_app/render/stop_range_ratio', self.init_view_stop_range_ratio)
            zoom_ratio = rospy.get_param('~iv_app/render/zoom_ratio',self.init_view_zoom_ratio)
            rotate_ratio = rospy.get_param('~iv_app/render/rotate_ratio',self.init_view_rotate_ratio)
            tilt_ratio = rospy.get_param('~iv_app/render/tilt_ratio',self.init_view_tilt_ratio)
            cam_fov = rospy.get_param('~iv_app/render/cam_fov', self.init_view_cam_fov )
            cam_view = rospy.get_param('~iv_app/render/cam_view',self.init_view_cam_view)
            cam_pos = rospy.get_param('~iv_app/render/cam_pos',self.init_view_cam_pos)
            cam_rot = rospy.get_param('~iv_app/render/cam_rot',self.init_view_cam_rot)

            # ToDo: Fix self img_min_range_m and img_max_range_m calcs
            min_range_m =  rospy.get_param('~iv_app/process/range_min_m', self.init_proc_range_min_m)
            max_range_m =   rospy.get_param('~iv_app/process/range_max_m', self.init_proc_range_max_m)


            delta_range_m = max_range_m - min_range_m
            self.clip_min_range_m = min_range_m + start_range_ratio  * delta_range_m
            self.clip_max_range_m = min_range_m + stop_range_ratio  * delta_range_m
            if start_range_ratio > 0 or stop_range_ratio < 1:
              o3d_img = nepi_img.range_clip( o3d_img, self.clip_min_range_m, self.clip_max_range_m)

            if cam_pos[0] < 0:
              zoom_ratio = 1 - zoom_ratio
            cam_pos[0] = cam_pos[0] *zoom_ratio  # Apply IDX zoom control

            rotate_angle = (0.5 - rotate_ratio) * 2 * 180
            rotate_vector = [0, 0, rotate_angle]
            o3d_img = nepi_img.rotate_img(o3d_img, rotate_vector)
            

            tilt_angle = (0.5 - tilt_ratio) * 2 * 180
            tilt_vector = [0, tilt_angle, 0]
            o3d_img = nepi_img.rotate_img(o3d_img, tilt_vector)
          
            ros_img_msg = None
            update_renderer = (self.img_renderer is None or self.img_renderer_mtl is None or self.last_img_width != img_width or self.last_img_height != img_height or self.last_fov != cam_fov)
            if update_renderer:
              # Create point cloud renderer
              self.img_renderer = nepi_img.create_img_renderer(img_width=img_width,img_height=img_height, fov=cam_fov, background = Render_Background)
              self.img_renderer_mtl = nepi_img.create_img_renderer_mtl()
              self.img_renderer = nepi_img.remove_img_renderer_geometry(self.img_renderer)
            else:
              self.img_renderer = nepi_img.add_img_renderer_geometry(o3d_img,self.img_renderer, self.img_renderer_mtl)
              o3d_img = nepi_img.render_img(self.img_renderer,cam_view,cam_pos,cam_rot)
              ros_img_msg = nepi_img.o3dimg_to_rosimg(o3d_img, stamp=ros_timestamp, frame_id=ros_frame_id)
              self.img_renderer = nepi_img.remove_img_renderer_geometry(self.img_renderer)
            self.last_img_width = img_width
            self.last_img_height = img_height
            self.last_fov = cam_fov


            # Publish and Save Pointcloud Image Data
            if ros_img_msg is not None:
              if img_has_subscribers:
                if not rospy.is_shutdown():
                  self.view_img_pub.publish(ros_img_msg)

              if img_saving_is_enabled is True or img_snapshot_enabled is True:
                cv2_img = nepi_img.rosimg_to_cv2img(ros_img_msg)
                self.save_img2file('pointcloud_image',cv2_img,ros_timestamp)
          
      else: # Data Empty
          rospy.sleep(0.1)
    else: # No data available
        rospy.sleep(0.25)
    rospy.sleep(0.01) # Yield
  
      
    
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
                                                                                              "pointcloud_app-" + data_product, 'png')
                      if os.path.isfile(full_path_filename) is False:
                          cv2.imwrite(full_path_filename, cv2_img)
                          self.save_data_if.data_product_snapshot_reset(data_product)

  def save_img2file(self,data_product,o3d_img,ros_timestamp):
      if self.save_data_if is not None:
          saving_is_enabled = self.save_data_if.data_product_saving_enabled(data_product)
          snapshot_enabled = self.save_data_if.data_product_snapshot_enabled(data_product)
          # Save data if enabled
          if saving_is_enabled or snapshot_enabled:
              if o3d_img is not None:
                  if (self.save_data_if.data_product_should_save(data_product) or snapshot_enabled):
                      full_path_filename = self.save_data_if.get_full_path_filename(nepi_ros.get_datetime_str_from_stamp(ros_timestamp), 
                                                                                              "pointcloud_app-" + data_product, 'imgd')
                      if os.path.isfile(full_path_filename) is False:
                          nepi_img.save_pointcloud(o3d_img,full_path_filename)
                          self.save_data_if.data_product_snapshot_reset(data_product)

                

  #######################
  # Utility Funcitons

  def getAvailableFrame3DList(self):
    frames_dict = yaml.safe_load(self.tf_buffer.all_frames_as_yaml())
    frame3d_list = list(frames_dict.keys())
    frame3d_list.insert(0,'map')
    return frame3d_list

  def getFrame3DTransformsMsg(self):
    transforms_dict = rospy.get_param('~iv_app/transforms_dict',self.init_transforms_dict)
    transforms_topic_list = []
    transforms_list = []
    for topic in transforms_dict.keys():
      transforms_topic_list.append(topic)
      transform=transforms_dict[topic]
      transforms_list.append(transform)
    return str(transforms_topic_list), str(transforms_list)

  def getTransformFromMsg(self,transform_msg):
    x = transform_msg.translate_vector.x
    y = transform_msg.translate_vector.y
    z = transform_msg.translate_vector.z
    roll = transform_msg.rotate_vector.x
    pitch = transform_msg.rotate_vector.y
    yaw = transform_msg.rotate_vector.z
    heading = transform_msg.heading_offset
    transform = [x,y,z,roll,pitch,yaw,heading]
    return transform

  def addTransformToDict(self,transform_msg):
    topic = transform_msg.topic_namespace
    transforms_dict = rospy.get_param('~iv_app/transforms_dict',self.init_transforms_dict)
    transforms_dict[topic] = self.getTransformFromMsg(transform_msg.transform)
    rospy.set_param('~iv_app/transforms_dict',transforms_dict)

  def removeTransformFromDict(self,topic_namespace):
    transforms_dict = rospy.get_param('~iv_app/transforms_dict',self.init_transforms_dict)
    if topic_namespace in transforms_dict:
      transforms_dict.pop(topic_namespace)
    rospy.set_param('~iv_app/transforms_dict',transforms_dict)

    
  #######################
  # Node Cleanup Function
  
  def cleanup_actions(self):
    rospy.loginfo("IV_APP:  Shutting down: Executing script cleanup actions")


#########################################
# Main
#########################################
if __name__ == '__main__':
  node_name = "pointcloud_app"
  rospy.init_node(name=node_name)
  #Launch the node
  rospy.loginfo("IV_APP:  Launching node named: " + node_name)
  node_class = eval(node_name)
  node = node_class()
  #Set up node shutdown
  rospy.on_shutdown(node.cleanup_actions)
  # Spin forever (until object is detected)
  rospy.spin()





