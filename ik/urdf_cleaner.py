import urdfpy
import numpy as np

import ikpy.urdf.utils
import ikpy.chain
import pathlib
from IPython import display
import ipywidgets as widgets
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot as plt
import stretch_body.hello_utils as hu
import time

class URDFConverter():
    def __init__(self, links_to_remove=['link_right_wheel', 'link_left_wheel', 'caster_link', 'link_gripper_finger_left', 'link_gripper_fingertip_left', 'link_gripper_finger_right', 'link_gripper_fingertip_right', 'link_head', 'link_head_pan', 'link_head_tilt', 'link_aruco_right_base', 'link_aruco_left_base', 'link_aruco_shoulder', 'link_aruco_top_wrist', 'link_aruco_inner_wrist', 'camera_bottom_screw_frame', 'camera_link', 'camera_depth_frame', 'camera_depth_optical_frame', 'camera_infra1_frame', 'camera_infra1_optical_frame', 'camera_infra2_frame', 'camera_infra2_optical_frame', 'camera_color_frame', 'camera_color_optical_frame', 'camera_accel_frame', 'camera_accel_optical_frame', 'camera_gyro_frame', 'camera_gyro_optical_frame', 'laser', 'respeaker_base'], joints_to_remove = ['joint_right_wheel', 'joint_left_wheel', 'caster_joint', 'joint_gripper_finger_left', 'joint_gripper_fingertip_left', 'joint_gripper_finger_right', 'joint_gripper_fingertip_right', 'joint_head', 'joint_head_pan', 'joint_head_tilt', 'joint_aruco_right_base', 'joint_aruco_left_base', 'joint_aruco_shoulder', 'joint_aruco_top_wrist', 'joint_aruco_inner_wrist', 'camera_joint', 'camera_link_joint', 'camera_depth_joint', 'camera_depth_optical_joint', 'camera_infra1_joint', 'camera_infra1_optical_joint', 'camera_infra2_joint', 'camera_infra2_optical_joint', 'camera_color_joint', 'camera_color_optical_joint', 'camera_accel_joint', 'camera_accel_optical_joint', 'camera_gyro_joint', 'camera_gyro_optical_joint', 'joint_laser', 'joint_respeaker']):
        
        self.path = ""
        self.links_to_remove = links_to_remove
        self.joints_to_remove = joints_to_remove
        self.urdf_path = '/home/hello-robot/replication_study/src/stretch_ros/stretch_description/urdf'

    def convertUrdf(self):
        """
        Converts URDF into ikpy ready state 
        """
 
        self.current_urdf()
        self.simplify_urdf()
        if self.modified_urdf is not None:
            self.save(self.modified_urdf)
        
        #Double check current tree
        #tree = ikpy.urdf.utils.get_urdf_tree(self.urdf_path, "base_link")[0]
        #tree.render()
        #display.display_png(tree)
        return True 

    def get_path(self):
        """
        Displays and grabs currently URDF paths
        """
        urdf_path = str((pathlib.Path(self.urdf_path).absolute()))
        tree = ikpy.urdf.utils.get_urdf_tree(urdf_path, "base_link")[0]
        #display.display_svg(tree)
        return urdf_path

    def current_urdf(self):
        """"
        Loads the currently stored URDF file
        """
        urdf_path = str((pathlib.Path(hu.get_fleet_directory()) / 'exported_urdf' / 'stretch.urdf').absolute())
        loaded_urdf = urdfpy.URDF.load(urdf_path)
        print(f"name: {loaded_urdf.name}")
        print(f"num links: {len(loaded_urdf.links)}")
        print(f"num joints: {len(loaded_urdf.joints)}")
        self.original_urdf = loaded_urdf

    def simplify_urdf(self):
        """
        Returns a simplified URDF chain
        """
        modified_urdf = self.original_urdf.copy()
        names_of_links_to_remove = self.links_to_remove 
        links_to_remove = [l for l in modified_urdf._links if l.name in names_of_links_to_remove]
        for lr in links_to_remove:
            modified_urdf._links.remove(lr)
        names_of_joints_to_remove = self.joints_to_remove
        joints_to_remove = [l for l in modified_urdf._joints if l.name in names_of_joints_to_remove]
        for jr in joints_to_remove:
            modified_urdf._joints.remove(jr)
        print(f"name: {modified_urdf.name}")
        print(f"num links: {len(modified_urdf.links)}")
        print(f"num joints: {len(modified_urdf.joints)}")
        self.modified_urdf = modified_urdf


    def save(self,modified_urdf):
        """
        Saves
        """
        iktuturdf_path = "/tmp/iktutorial/stretch.urdf"
        modified_urdf.save(iktuturdf_path)
        tree = ikpy.urdf.utils.get_urdf_tree(iktuturdf_path, "base_link")[0]
        display.display_svg(tree)
        return iktuturdf_path
    
cleaner = URDFConverter()
cleaner.convertUrdf()