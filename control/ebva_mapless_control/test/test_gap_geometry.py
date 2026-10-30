from types import SimpleNamespace
import pytest
from ebva_mapless_control.scan_direct import FollowTheGap as Direct
from ebva_mapless_control.scan_pd import FollowTheGap as ScanPD
from ebva_mapless_control.ftg_pd_core import FtgPdCore as AutoPD
from ebva_mapless_control.gap_geometry import GapGeometry


@pytest.mark.parametrize('cls',[Direct,ScanPD,AutoPD])
def test_real_consumers_share_gap_geometry(cls):
    assert cls.findDisparities is GapGeometry.findDisparities
    assert cls.find_consecutive_samples is GapGeometry.find_consecutive_samples
    fake=SimpleNamespace(disparityTreshold=1.)
    assert cls.findDisparities(fake,SimpleNamespace(ranges=[1.,1.,3.,3.]))==[1]
    assert cls.find_consecutive_samples(None,[0,2,2,0,3,3],2,2)==[1,2,4,5]


@pytest.mark.parametrize('cls',[Direct,ScanPD,AutoPD])
@pytest.mark.parametrize('ranges,expected', [([float('inf'),float('-inf')],[10.,10.]),([float('nan'),2.],[10.,2.])])
def test_all_infinite_scan_no_longer_crashes_preprocessing(cls,ranges,expected):
    scan=SimpleNamespace(ranges=ranges,range_max=10.)
    result=cls.lidarPreprocessing(None,scan)
    assert result.ranges==expected
