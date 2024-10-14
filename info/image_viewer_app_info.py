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
FILE_TYPE = 'APP'
APP_DICT = dict(
    description = 'Application for viewing and saving multiple image streams',
    pkg_name = 'nepi_app_image_viewer',
    group_name = 'IMAGING',
    config_file = 'app_image_viewer.yaml',
    app_file = 'image_viewer_app_node.py',
    node_name = 'app_image_viewer'
)
RUI_DICT = dict(
    rui_menu_name = 'Image Viewer', # RUI menu name or "None" if no rui support
    rui_files = ['NepiAppImageViewer.js'],
    rui_main_file ='NepiAppImageViewer.js',
    rui_main_class = 'ImageViewerApp'
)


