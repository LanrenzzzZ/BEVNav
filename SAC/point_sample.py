import numpy as np
from numpy import inf

def regularize_pc(points, sample_size=5200):
    """
    points: numpy.array [N, 3]
    """
    # points = crop_pc(points)
    num_points = points.shape[0]
    new_pts_idx = None
    if num_points > 2:
        if num_points != sample_size:
            new_pts_idx = np.random.choice(num_points, size=sample_size - 1, replace=sample_size > num_points)
        else:
            new_pts_idx = np.arange(num_points)
    if new_pts_idx is not None:
        points = points[new_pts_idx, :]
    else:
        points = np.zeros((sample_size - 1, 3), dtype='float32')
    noise = np.zeros((1, 3), dtype='float32')
    points = np.concatenate((noise, points), axis=0)
    return points

def crop_pc(points, sample_size):
    points = regularize_pc(points, sample_size)
    # vis(points)
    return points


if __name__ == "__main__":
    points = np.random.randn(512, 3)
    # crop_pc(points)
    print(crop_pc(points, 512).shape)