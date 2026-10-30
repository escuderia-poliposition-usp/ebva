from pathlib import Path
import cv2
import numpy as np
import pytest
from ebva_calibration.configuration import load_configuration
from ebva_calibration.intrinsic_math import checkerboard_points,fit_intrinsics


def test_synthetic_calibration_recovers_camera_geometry():
    points=checkerboard_points(6,9,.03)
    camera=np.array([[600.,0,320.],[0,610.,240.],[0,0,1.]])
    objects=[];images=[]
    for i in range(12):
        rotation=np.array([.03*(i-5),.02*(i%4-2),.015*i])
        translation=np.array([-.1+.01*i,-.1+.005*i,.8+.04*(i%3)])
        pixels,_=cv2.projectPoints(points,rotation,translation,camera,np.zeros(5))
        objects.append(points);images.append(pixels)
    rms,matrix,dist,*_=fit_intrinsics(objects,images,(640,480))
    assert rms<1e-3
    np.testing.assert_allclose(matrix,camera,rtol=1e-3,atol=.1)


def test_requires_real_view_count_and_geometry():
    with pytest.raises(ValueError):checkerboard_points(1,9,.03)
    with pytest.raises(ValueError):fit_intrinsics([],[],(640,480))


def test_explicit_configuration_and_relative_output(tmp_path):
    with pytest.raises(ValueError):load_configuration('', 'intrinsic')
    path=tmp_path/'config.yaml'
    path.write_text("camera:\n  camera_topic: /image\n  image_size: {width: 640, length: 480}\n  chessboard: {rows: 6, columns: 9, square_size_m: .03}\ngeneral:\n  config_folder: results\n  camera_intrinsic_calibration: camera.yaml\n")
    config=load_configuration(path,'intrinsic')
    assert config['general']['config_folder']==str(tmp_path/'results')
    assert config['chessboard']['rows']==6
