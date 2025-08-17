"""
# Created: 2024-11-20 13:11
# Copyright (C) 2024-now, RPL, KTH Royal Institute of Technology
# Author: Qingwen Zhang  (https://kin-zhang.github.io/)
#
# This file is part of DUFOMap (https://github.com/KTH-RPL/dufomap) and 
# DynamicMap Benchmark (https://github.com/KTH-RPL/DynamicMap_Benchmark) projects.
# If you find this repo helpful, please cite the respective publication as 
# listed on the above website.

# Description: Output Cleaned Map through Python API.
"""
from pathlib import Path
import os, fire, time
import numpy as np
from tqdm import tqdm

from dufomap import dufomap
from dufomap.utils import pcdpy3
def inv_pose_matrix(pose):
    inv_pose = np.eye(4)
    inv_pose[:3, :3] = pose[:3, :3].T
    inv_pose[:3, 3] = -pose[:3, :3].T.dot(pose[:3, 3])
    return inv_pose

MIN_AXIS_RANGE = 0.2 # HARD CODED: remove ego vehicle points
MAX_AXIS_RANGE = 50 # HARD CODED: remove far away points

class DynamicMapData:
    def __init__(self, directory):
        self.scene_id = directory.split("/")[-1]
        self.directory = Path(directory) / "pcd"
        self.pcd_files = [os.path.join(self.directory, f) for f in sorted(os.listdir(self.directory)) if f.endswith('.pcd')]

    def __len__(self):
        return len(self.pcd_files)
    
    def __getitem__(self, index_):
        res_dict = {
            'scene_id': self.scene_id,
            'timestamp': self.pcd_files[index_].split("/")[-1].split(".")[0],
        }
        pcd_ = pcdpy3.PointCloud.from_path(self.pcd_files[index_])
        pc0 = pcd_.np_data[:,:3]
        res_dict['pc'] = pc0.astype(np.float32)
        res_dict['pose'] = list(pcd_.viewpoint)
        return res_dict

def main_vis(
    data_dir: str = "/home/kin/data/00",
    voxel_map: bool = True, # output voxel-level map or raw point-level.
):
    dataset = DynamicMapData(data_dir)

    # STEP 0: initialize 
    mydufo = dufomap(0.1, 0.2, 2, num_threads=12) # resolution, d_s, d_p same with paper.
    cloud_acc = np.zeros((0, 3), dtype=np.float32)
    for data_id in (pbar := tqdm(range(0, len(dataset)),ncols=100)):
        data = dataset[data_id]
        now_scene_id = data['scene_id']
        pbar.set_description(f"id: {data_id}, scene_id: {now_scene_id}, timestamp: {data['timestamp']}")
        norm_pc0 = np.linalg.norm(data['pc'][:, :3] - data['pose'][:3], axis=1)
        range_mask = (
                (norm_pc0>MIN_AXIS_RANGE) & 
                (norm_pc0<MAX_AXIS_RANGE)
        )
        # STEP 1: integrate point cloud into dufomap
        mydufo.run(data['pc'][range_mask], data['pose'], cloud_transform = False)
        # STEP 1: integrate point cloud into dufomap
        cloud_acc = np.concatenate((cloud_acc, data['pc']), axis=0)
    
    # STEP 2: propagate
    mydufo.oncePropagateCluster(if_propagate=True, if_cluster=False)
    # STEP 3: Map results; You can save the voxel map directly based on the resolution we set before:
    mydufo.outputMap(cloud_acc, voxel_map=voxel_map)
    
    mydufo.printDetailTiming()

if __name__ == "__main__":
    start_time = time.time()
    fire.Fire(main_vis)
    print(f"Time used: {time.time() - start_time:.2f} s")