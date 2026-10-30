"""Image envelope contract; downstream owns measurement freshness and lane geometry policy."""
import pytest
import numpy as np
from cv_bridge import CvBridge
from ebva_lane_detection.image_contract import validate_image_message


def image(encoding="bgr8"):
    channels = {"bgr8":3,"rgb8":3,"mono8":1,"bgra8":4,"rgba8":4}[encoding]
    shape = (30,40) if channels == 1 else (30,40,channels)
    msg = CvBridge().cv2_to_imgmsg(np.zeros(shape,np.uint8),encoding=encoding)
    msg.header.stamp.sec = 10; msg.header.stamp.nanosec = 123
    msg.header.frame_id = "fixture_image_optical"
    return msg


@pytest.mark.parametrize("encoding", ["bgr8","rgb8","mono8","bgra8","rgba8"])
def test_explicit_8bit_encodings(encoding):
    assert validate_image_message(image(encoding)) == 10_000_000_123


@pytest.mark.parametrize("change,reason", [
    (lambda m:setattr(m.header.stamp,"sec",-1),"noncanonical_measurement_stamp"),
    (lambda m:setattr(m.header.stamp,"nanosec",1_000_000_000),"noncanonical_measurement_stamp"),
    (lambda m:setattr(m.header,"frame_id",""),"missing_or_invalid_input_frame"),
    (lambda m:setattr(m.header,"frame_id"," bad "),"missing_or_invalid_input_frame"),
    (lambda m:setattr(m,"encoding","16UC1"),"unsupported_image_encoding"),
    (lambda m:setattr(m,"height",1),"invalid_image_dimensions"),
    (lambda m:setattr(m,"step",1),"invalid_image_step"),
    (lambda m:setattr(m,"data",m.data[:-1]),"invalid_image_data_length"),
])
def test_malformed_envelope_rejected_with_reason(change,reason):
    msg = image(); change(msg)
    with pytest.raises(ValueError,match=reason): validate_image_message(msg)


def test_original_envelope_not_modified_and_padded_rows_accepted():
    msg = image(); msg.step += 4
    msg.data = bytes(msg.height*msg.step)
    before = bytes(msg.data)
    assert validate_image_message(msg) == 10_000_000_123
    assert msg.header.stamp.nanosec == 123 and bytes(msg.data) == before
