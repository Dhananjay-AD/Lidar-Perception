from dataset import KittiDataset
from visualization import Visualization

def main(mode ,load , root_directory):
    dataset = KittiDataset(root_directory, 'train')
    visual = Visualization(dataset, load)
    if mode == 'load':
        if load == 'all':
            # load all data
            frame = {}
            for id in dataset.file_ids:
                frame[id] = dataset[id]
        else:
            frame = dataset[load]
    elif mode == 'visualization': 
        visual.visualize(visual.points, visual.labels)





if __name__ == "__main__":
    ROOT_DIR = "/home/dhananjay/Documents/Lidar-Perception/machine_learning_lidar_detection/dataset"
    main('visualization', '000454', ROOT_DIR)

        