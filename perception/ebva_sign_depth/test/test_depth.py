from types import SimpleNamespace
import cv2
import numpy as np
import pytest
from ebva_sign_depth.depth_units import depth_metres
from ebva_sign_depth.depth_node import profundidade_media_no_contorno,PlacaDetectorNode
from ebva_sign_depth.segmentation import segmentar_placa


def test_depth_units_and_invalid_samples():
    mm=np.array([[1000,0],[2000,0]],dtype=np.uint16)
    metres=depth_metres(mm,'16UC1')
    np.testing.assert_allclose(metres,[[1,0],[2,0]])
    assert mm[0,0]==1000
    contour=np.array([[[0,0]],[[1,0]],[[1,1]],[[0,1]]],np.int32)
    assert profundidade_media_no_contorno(metres,contour)==pytest.approx(1.5)
    assert profundidade_media_no_contorno(np.full((2,2),np.nan),contour) is None
    assert depth_metres(metres,'32FC1') is metres
    with pytest.raises(ValueError):depth_metres(mm,'rgb8')


def test_pinhole_distance_rejects_unusable_calibration():
    node=SimpleNamespace(fx=600.,diametro_real_m=.3)
    assert PlacaDetectorNode.estimar_distancia_por_tamanho(node,60)==pytest.approx(3.)
    assert PlacaDetectorNode.estimar_distancia_por_tamanho(node,0) is None
    node.fx=float('nan')
    assert PlacaDetectorNode.estimar_distancia_por_tamanho(node,60) is None


def test_synthetic_sign_and_empty_image():
    black=np.zeros((200,200,3),np.uint8)
    mask,contours,diameters=segmentar_placa(black)
    assert not contours and not np.any(mask)
    image=black.copy();cv2.circle(image,(100,100),50,(0,0,255),-1)
    cv2.circle(image,(100,100),40,(255,255,255),-1)
    mask,contours,diameters=segmentar_placa(image)
    assert len(contours)==1 and 90<diameters[0]<110 and mask[100,100]>0
