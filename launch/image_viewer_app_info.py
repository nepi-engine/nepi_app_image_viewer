#!/usr/bin/env python
#
# Copyright (c) 2024 Numurus, LLC <https://www.numurus.com>.
#
# This file is part of nepi-engine
# (see https://github.com/nepi-engine).
#
# License: 3-clause BSD, see https://opensource.org/licenses/BSD-3-Clause
#

APP_NAME = 'Image_Viewer' # Use in display menus
DESCRIPTION = 'Application for viewing and saving multiple image streams'
PKG_NAME = 'nepi_app_image_viewer'
APP_FILE = 'image_viewer_app_node.py'
NODE_NAME = 'app_image_viewer'
RUI_FILES = ['NepiAppImageViewer.js']
RUI_MAIN_FILE = "NepiAppImageViewer.js"
RUI_MAIN_CLASS = "ImageViewerApp"
RUI_MENU_NAME = "Image Viewer"

