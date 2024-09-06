import os
import open3d as o3d
import numpy as np
import cv2
import time
# /home/jjh/DRL-robot-navigation/TD3
dirPath = '/home/jjh/DRL-robot-navigation/TD3'
k = np.array([[121.79914855957031, 0.0, 112.0], [0.0, 121.79914855957031, 112.0], [0.0, 0.0, 1.0]])
intrinsic = o3d.camera.PinholeCameraIntrinsic(100, 100, k[0, 0], k[1, 1], k[0, 2], k[1, 2])
depth = cv2.imread('depth.png', cv2.IMREAD_UNCHANGED)

rgb = cv2.imread('rgb.png')
rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
    o3d.geometry.Image(rgb),
    o3d.geometry.Image(depth)
)

rgbd_to_pcd = o3d.geometry.PointCloud.create_from_rgbd_image(rgbd_image, intrinsic)

save_directory = dirPath + '/point_cloud'
timestep = int(time.time())
file_extension = ".png"
unique_filename = str(timestep) + file_extension
save_path = os.path.join(save_directory, unique_filename)

visualizer = o3d.visualization.Visualizer()
visualizer.create_window()
visualizer.add_geometry(rgbd_to_pcd)
visualizer.run()
visualizer.capture_screen_image(save_path)
visualizer.destroy_window()