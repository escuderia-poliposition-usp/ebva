"""ROS depth image encodings per REP 118; output is camera-Z metres."""
import numpy as np


def depth_metres(image,encoding):
    image=np.asarray(image)
    if image.ndim!=2:raise ValueError('Depth image must have one channel')
    if encoding=='32FC1' and image.dtype==np.float32:
        return image
    if encoding=='16UC1' and image.dtype==np.uint16:
        return image.astype(np.float32)*.001
    raise ValueError('Expected depth encoding 32FC1 [m] or 16UC1 [mm] with matching dtype')
