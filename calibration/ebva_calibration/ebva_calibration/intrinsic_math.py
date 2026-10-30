"""Checkerboard object geometry and OpenCV intrinsic fit."""
import numpy as np
import cv2


def checkerboard_points(rows,columns,square_size):
    if rows<2 or columns<2 or not np.isfinite(square_size) or square_size<=0:
        raise ValueError('Invalid checkerboard')
    points=np.zeros((rows*columns,3),np.float32)
    points[:,:2]=np.mgrid[0:columns,0:rows].T.reshape(-1,2)
    return points*square_size


def fit_intrinsics(objects,images,image_size):
    if len(objects)<10 or len(objects)!=len(images):raise ValueError('At least ten matched views are required')
    result=cv2.calibrateCamera(objects,images,image_size,None,None)
    if not np.isfinite(result[0]) or not np.isfinite(result[1]).all() or not np.isfinite(result[2]).all():
        raise ValueError('Nonfinite calibration result')
    return result
