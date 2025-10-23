import numpy as np
import open3d as o3d
import os
from dataset import KittiDataset

class Visualization:
    def __init__(self, dataset: KittiDataset, file_id):
        self.dataset = dataset
        self.points, self.labels, self.calib = self.dataset[file_id]

    def create_pcl(self, pcl_points):
        pcl = o3d.geometry.PointCloud()
        pcl.points = o3d.utility.Vector3dVector(pcl_points[:,:3])
        intensity = pcl_points[:,3].reshape(-1, 1)
        # normalization of intensity
        intensity = (intensity - np.min(intensity))/(np.max(intensity) - np.min(intensity))
        pcl.colors = o3d.utility.Vector3dVector(np.hstack((intensity, intensity, intensity)))
        return pcl

    # todo: add the color feature for bbox
    def create_3D_boxes(self, label):
        # loading labels for visualization
        bbox = o3d.geometry.OrientedBoundingBox()
        class_name, x, y, z, l, w, h, rotation = label
        print(rotation)
        bbox.center = [x, y, z + w/2]
        bbox.extent = [h, l, w]
        bbox.R = rotation
        bbox.color = [0, 0, 1]
        return bbox
    
    def visualize(self, pcl_points, labels):
        pcl = self.create_pcl(pcl_points)
        bboxes = [self.create_3D_boxes(label) for label in labels]
        o3d.visualization.draw_geometries([pcl,*bboxes])


        

        





