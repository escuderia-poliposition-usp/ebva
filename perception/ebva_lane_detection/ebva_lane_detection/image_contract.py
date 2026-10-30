"""Image envelope checks; pixels still use cv_bridge's explicit bgr8 conversion."""
SUPPORTED_ENCODINGS = {"bgr8": 3, "rgb8": 3, "mono8": 1, "bgra8": 4, "rgba8": 4}


def validate_image_message(msg):
    """Return canonical measurement ns; no frame transform or age refresh."""
    sec, ns = msg.header.stamp.sec, msg.header.stamp.nanosec
    if sec < 0 or not 0 <= ns < 1_000_000_000 or sec * 1_000_000_000 + ns <= 0:
        raise ValueError("noncanonical_measurement_stamp")
    if not msg.header.frame_id or msg.header.frame_id.strip() != msg.header.frame_id:
        raise ValueError("missing_or_invalid_input_frame")
    if msg.encoding not in SUPPORTED_ENCODINGS:
        raise ValueError("unsupported_image_encoding")
    if msg.height < 2 or msg.width < 2:
        raise ValueError("invalid_image_dimensions")
    if msg.step < msg.width * SUPPORTED_ENCODINGS[msg.encoding]:
        raise ValueError("invalid_image_step")
    if len(msg.data) != msg.height * msg.step:
        raise ValueError("invalid_image_data_length")
    return sec * 1_000_000_000 + ns
