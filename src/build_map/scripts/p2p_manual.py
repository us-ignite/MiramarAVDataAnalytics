#!/usr/bin/env python3

# This program generates 3D pointcloud map from lidar data of the avaialable lidars mounted on the vehicle. For now, only the front two lidars data is utilized to generate the 3D map since adding 
# rear lidar data made the generation proccess slower & did not show any improvement in the quality of map generated. 

import open3d as o3d
import copy
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2
import ros_numpy
import tf2_ros
import time

class build_map:
    def __init__(self):
        # Variables for accumulating point clouds over time
        self.map_combined = o3d.geometry.PointCloud()
        self.map_fr_upr = o3d.geometry.PointCloud()
        self.map_fr_lwr = o3d.geometry.PointCloud()
        self.map_rr_upr = o3d.geometry.PointCloud()

        # Variables for counter & time to combine aps after a certain duration
        self.n = 0
        self.start_time = time.time()
        try:
            self.map_duration = rospy.get_param('~duration') - 10.0
        except Exception:
            print('Choosing default map-build duration value of 10.0 secs')
            self.map_duration = 10.0

        #Variables for storing instantaneous point clouds for each lidar & immediate previous version of the point clouds
        self.fr_lwr_pc = None
        self.fr_lwr_pc_old = None
        self.fr_upr_pc = None
        self.fr_upr_pc_old = None
        self.rr_upr_pc = None
        self.rr_upr_pc_old = None

        # Parameters used for ICP registration algorithm
        self.voxel_size = 1.0
        self.threshold = self.voxel_size * 1.5

        # Variables related to Transforms including - 
        # 1. Defining Name of absolute frame
        # 2. Initial guess of Transform matrix for ICP
        # 3. Creating  object for TF listener to get transforms between defined frames on call later in program
        # 4. Defining topics for which the transform is required in the program
        self.tf_abs_frm = 'absolute_frd'
        self.tf_init = None
        self.tfBuffer = tf2_ros.Buffer()
        self.tflistener = tf2_ros.TransformListener(self.tfBuffer)
        self.tf_fr_lwr_frm = 'vlp16_front_lower_frd'
        self.tf_fr_upr_frm = 'vlp16_front_upper_frd'
        self.tf_rr_upr_frm = 'vlp16_rear_upper_frd'
        
        # Topic Names & their Subscrbers for getting data published on each topic
        self.pc_rr_upr_topic = '/perception/vel16/rear_upper'
        self.rear_upr_sub = rospy.Subscriber(self.pc_rr_upr_topic, PointCloud2, self.rear_upr_callback)
        self.pc_fr_lwr_topic = '/perception/vel16/front_lower'
        self.front_lwr_sub = rospy.Subscriber(self.pc_fr_lwr_topic, PointCloud2, self.front_lwr_callback)
        self.pc_fr_upr_topic = '/perception/vel16/front_upper'
        self.front_upr_sub = rospy.Subscriber(self.pc_fr_upr_topic, PointCloud2, self.front_upr_callback)

    # A function that converts quaternion coordinates to roatation matrix to be used later to calculate the transformation matrix
    def quaternion_rotation_matrix(self,w, x, y, z):
        """
        Covert a quaternion into a full three-dimensional rotation matrix.
    
        Input
        :param Q: A 4 element array representing the quaternion (q0,q1,q2,q3) 
    
        Output
        :return: A 3x3 element matrix representing the full 3D rotation matrix. 
                This rotation matrix converts a point in the local reference 
                frame to a point in the global reference frame.
        """
        # Extract the values from Q
        q0 = w
        q1 = x
        q2 = y
        q3 = z
        
        # First row of the rotation matrix
        r00 = 2 * (q0 * q0 + q1 * q1) - 1
        r01 = 2 * (q1 * q2 - q0 * q3)
        r02 = 2 * (q1 * q3 + q0 * q2)
        
        # Second row of the rotation matrix
        r10 = 2 * (q1 * q2 + q0 * q3)
        r11 = 2 * (q0 * q0 + q2 * q2) - 1
        r12 = 2 * (q2 * q3 - q0 * q1)
        
        # Third row of the rotation matrix
        r20 = 2 * (q1 * q3 - q0 * q2)
        r21 = 2 * (q2 * q3 + q0 * q1)
        r22 = 2 * (q0 * q0 + q3 * q3) - 1
        
        # 3x3 rotation matrix
        rot_matrix = np.array([[r00, r01, r02],
                            [r10, r11, r12],
                            [r20, r21, r22]])
                                
        return rot_matrix

    # A function that takes 2 frames between which the transforms are to be calculated & returns the trasnfromation matrix for the transform using the translation & rotation info from TF message
    def get_tf(self,abs_frm,target_frm):
        tf_msg = self.tfBuffer.lookup_transform(abs_frm, target_frm, rospy.Time(0))
        tf_rot_mat = self.quaternion_rotation_matrix(tf_msg.transform.rotation.w, tf_msg.transform.rotation.x, tf_msg.transform.rotation.y, tf_msg.transform.rotation.z)
        tf_trans_mat = np.array([[tf_msg.transform.translation.x], [tf_msg.transform.translation.y], [tf_msg.transform.translation.z]])
        tf_transform = np.vstack((np.hstack((tf_rot_mat, tf_trans_mat)), [0, 0, 0 ,1]))
        if self.n == 0:
            self.tf_init = tf_transform
        return tf_transform

    def rear_upr_callback(self,msg):
        self.tf_rr_upr = self.get_tf(self.tf_abs_frm,self.tf_rr_upr_frm)
        self.rr_upr_pc = o3d.geometry.PointCloud()
        # Transform the point cloud points to the point cloud points object that Open3D library supports
        self.rr_upr_pc.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg))
        if self.n == 0:
            self.rr_upr_pc_old = copy.deepcopy(self.rr_upr_pc)
        else:
            source_rr_upr = copy.deepcopy(self.rr_upr_pc)
            source_rr_upr.transform(self.tf_rr_upr)
            # Accumulate multiple point clouds over time from the front lower sensor 
            self.map_rr_upr += source_rr_upr
            
            if self.map_duration < (time.time() - self.start_time) < (self.map_duration + 0.2) :
                pass
                # o3d.visualization.draw_geometries([self.map_rr_upr],
                #                       zoom=0.4459,
                #                       front=[0.9288, -0.2951, -0.2242],
                #                       lookat=[1.6784, 2.0612, 1.4451],
                #                       up=[-0.3402, -0.9189, -0.1996])
                #To save the accumulated point cloud generated from single sensor
                # o3d.io.write_point_cloud("map_rr_upr.pcd", self.map_rr_upr)
        self.rr_upr_pc_old = copy.deepcopy(self.rr_upr_pc)

    # Callback function for scan from front Lower Lidar  
    def front_lwr_callback(self, msg):
        self.tf_fr_lwr = self.get_tf(self.tf_abs_frm,self.tf_fr_lwr_frm)
        self.fr_lwr_pc = o3d.geometry.PointCloud()
        # Transform the point cloud points to the point cloud points object that Open3D library supports
        self.fr_lwr_pc.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg))
        if self.n == 0:
            self.fr_lwr_pc_old = copy.deepcopy(self.fr_lwr_pc)
            self.n += 1
        else:
            source_fr_lwr = copy.deepcopy(self.fr_lwr_pc)
            source_fr_lwr.transform(self.tf_fr_lwr)
            # Accumulate multiple point clouds over time from the front lower sensor 
            self.map_fr_lwr += source_fr_lwr
            
            if self.map_duration < (time.time() - self.start_time) < (self.map_duration + 0.2) :
                pass
                #To save the accumulated point cloud generated from single sensor
                # o3d.io.write_point_cloud("map_fr_lwr.pcd", self.map_fr_lwr)
        self.fr_lwr_pc_old = copy.deepcopy(self.fr_lwr_pc)

    # Callback function for scan from front upper lidar. Also, combines the point cloud from both front lidars to form a single pointcloud map. 
    def front_upr_callback(self, msg):
        self.tf_fr_upr = self.get_tf(self.tf_abs_frm,self.tf_fr_upr_frm)
        self.fr_upr_pc = o3d.geometry.PointCloud()
        self.fr_upr_pc.points = o3d.utility.Vector3dVector(ros_numpy.point_cloud2.pointcloud2_to_xyz_array(msg))
        if self.n == 0:
            self.fr_upr_pc_old = copy.deepcopy(self.fr_upr_pc)
            self.n += 1
        else:
            source_fr_upr = copy.deepcopy(self.fr_upr_pc)
            source_fr_upr.transform(self.tf_fr_upr)
            self.map_fr_upr += source_fr_upr
            
            if self.map_duration < (time.time() - self.start_time) < (self.map_duration + 0.2) :
                # o3d.io.write_point_cloud("map_fr_upr.pcd", self.map_fr_upr)
                self.combine_maps()
                
        self.fr_upr_pc_old = copy.deepcopy(self.fr_upr_pc)

    # A function that combines pointclouds from both the lidar sensors & then applies ICP P2P registration for aligning the 2 pointclouds.
    def combine_maps(self):
        fr_lwr_cpy = copy.deepcopy(self.map_fr_upr)
        fr_upr_cpy = copy.deepcopy(self.map_fr_lwr)
        # rr_upr_cpy = copy.deepcopy(self.map_rr_upr)
        tf_com_trans_init_front = self.get_tf('vlp16_front_lower_frd', 'vlp16_front_upper_frd')
        reg_p2p_front = self.p2p_icp(fr_lwr_cpy, fr_upr_cpy, tf_com_trans_init_front)
        self.map_fr_upr.transform(reg_p2p_front.transformation)
        self.map_combined = self.map_fr_upr + self.map_fr_lwr #+ self.map_rr_upr
        # o3d.io.write_point_cloud("map_fr_com_icp.pcd", self.map_combined)
        o3d.visualization.draw_geometries([self.map_combined],
                                      zoom=0.4459,
                                      front=[0.9288, -0.2951, -0.2242],
                                      lookat=[1.6784, 2.0612, 1.4451],
                                      up=[-0.3402, -0.9189, -0.1996])

        
    # A function for using ICP P2P  registration algorithm supplied by Open3D (Version - 0.13.0) library (http://www.open3d.org/docs/0.13.0/tutorial/pipelines/icp_registration.html)
    def p2p_icp(self, fr_lwr_cpy, fr_upr_cpy, tf_com_trans_init):
        fr_lwr_cpy = fr_lwr_cpy.voxel_down_sample(self.voxel_size*0.25)
        fr_upr_cpy = fr_upr_cpy.voxel_down_sample(self.voxel_size*0.25)
        reg_p2p = o3d.pipelines.registration.registration_icp(fr_lwr_cpy, fr_upr_cpy, self.threshold, tf_com_trans_init, o3d.pipelines.registration.TransformationEstimationPointToPoint(),o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=2000))
        return reg_p2p

def main():
    rospy.init_node('Manual',anonymous=True)
    map = build_map()
    rospy.spin()

if __name__ == '__main__':
    main()



