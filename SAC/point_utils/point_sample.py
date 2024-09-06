import numpy as np
from numpy import inf

def regularize_pc(points, sample_size=1024):
    """
    points: numpy.array [N, 3]
    """
    # points = crop_pc(points)
    num_points = points.shape[0]
    new_pts_idx = None
    if num_points > 2:
        if num_points != sample_size:
            new_pts_idx = np.random.choice(num_points, size=sample_size, replace=sample_size > num_points)
        else:
            new_pts_idx = np.arange(num_points)
    if new_pts_idx is not None:
        points = points[new_pts_idx, :]
    else:
        points = np.zeros((sample_size, 3), dtype='float32')
    return points

def pr_normalize(points):
    centroid = np.mean(points, axis=0)
    points = points - centroid
    m = np.max(np.sqrt(np.sum(points ** 2, axis=1)))
    pc = points / m
    return pc


def crop_pc(points):
    points = regularize_pc(points, 2048)
    return points


if __name__ == "__main__":
    points = np.random.randn(512, 3)
    crop_pc(points)