import pytest
from ebva_control_contracts import MeasurementLease


def test_repeated_measurement_and_frozen_ros_clock_cannot_extend_local_lifetime():
    lease=MeasurementLease(.2,"body")
    assert lease.observe(10.,10.1,100.,"body")
    assert not lease.observe(10.,10.1,100.05,"body")
    assert lease.fresh(10.1,100.09)
    assert not lease.fresh(10.1,100.11)


def test_oldest_input_remains_bounded_independently_of_new_primary_inputs():
    feedback=MeasurementLease(.2,"body");scan=MeasurementLease(.2,"body")
    assert feedback.observe(10.,10.,100.,"body")
    for n in range(1,5):assert scan.observe(10.+n*.01,10.+n*.01,100.+n*.05,"body")
    assert not feedback.fresh(10.04,100.21)
    assert scan.fresh(10.04,100.21)


@pytest.mark.parametrize("stamp,frame",[(0.,"body"),(9.,"body"),(10.1,"body"),(10.,"wrong")])
def test_invalid_measurement_never_starts_validity(stamp,frame):
    lease=MeasurementLease(.2,"body")
    assert not lease.observe(stamp,10.,100.,frame)
    assert not lease.fresh(10.,100.)
