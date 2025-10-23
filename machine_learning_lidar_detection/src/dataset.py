import numpy as np
import os
from scipy.spatial.transform import Rotation as R

class KittiDataset:
    def __init__(self,root_dir, mode):
        self.root_dir = root_dir
        self.velodyne_train_dir = os.path.join(root_dir,"data_object_velodyne/training/velodyne")
        self.velodyne_test_dir = os.path.join(root_dir,"data_object_velodyne/testing/velodyne")
        self.label_dir = os.path.join(root_dir,"data_object_label_2/training/label_2")
        self.calib_train_dir = os.path.join(root_dir,"data_object_calib/training/calib")
        self.calib_test_dir = os.path.join(root_dir,"data_object_calib/testing/calib")

        if mode == 'train':
            self.file_ids = [f.split('.')[0] for f in sorted(os.listdir(self.velodyne_train_dir)) if f.endswith('.bin')]
        elif mode == 'test':
            self.file_ids = [f.split('.')[0] for f in sorted(os.listdir(self.velodyne_test_dir)) if f.endswith('.bin')]

    def __len__(self):
        return len(self.file_ids)
    
    def __getitem__(self, file_id):
        """ returns pointcloud, labels and calibration matrices of file_id"""
        if isinstance(file_id, int):
            file_id = self.file_ids[file_id]
        calib = self.load_calib_dataset(file_id)
        points = self.load_velodyne_dataset(file_id)
        label = self.load_label_dataset(file_id)
        transformed_label = self.transformation_camera_to_velodyne(calib, label)
        return points, transformed_label, calib
        


    def load_velodyne_dataset(self, file_id):
        """
        Load the lidar point cloud dataset from velodyne
        """
        points = np.fromfile(os.path.join(self.velodyne_train_dir, file_id + '.bin' ), dtype = np.float32).reshape(-1,4)
        return points
        

    def load_label_dataset(self, file_id):
        """
        Load the labels 
        """
        label_directory = os.path.join(self.label_dir, file_id + '.txt')
        labels = []
        with open(label_directory, 'r') as f:
            for line in f:
                label = line.strip().split()
                class_name = label[0]
                l, w, h, x, y, z, ry = map(float, label[8:15])
                if class_name == 'DontCare':
                    continue
                labels.append([class_name, x, y, z, l, w, h, ry])  
        return labels 

    def load_calib_dataset(self, file_id):
        calib_directory = os.path.join(self.calib_train_dir, file_id + '.txt')
        calib = {}
        with open(calib_directory, 'r') as f:
            for line in f:
                if ':' in line:
                    key, value = line.strip().split(':', 1)
                    if key == 'R0_rect':
                        value = np.array(value.strip().split(), dtype=float).reshape(3,3)
                        value = np.hstack((value, np.zeros((3,1))))
                        value = np.vstack((value, [0,0,0,1]))
                    elif key == 'Tr_velo_to_cam':
                        value = np.array(value.strip().split(), dtype=float).reshape(3, 4)
                        value = np.vstack((value, [0, 0, 0, 1]))
                    calib[key] = value
        return calib

    def transformation_camera_to_velodyne(self, calib, label):
        transformed_label = []
        T = np.linalg.inv(calib['R0_rect']@calib['Tr_velo_to_cam'])
        R_velo = T[:3,:3]
        for lbl in label:
            x, y, z, l , w , h, ry = lbl[1:8]
            class_name = lbl[0]
            point = np.array([x, y, z, 1])
            transformed_point = T@point
            x, y, z = transformed_point[0:3]
            rotation = R.from_euler('xyz',[0,ry,0],degrees = False)
            rotation = R_velo@rotation.as_matrix()
            transformed_label.append([class_name, x, y, z, l, w, h, rotation])
        return transformed_label
    





