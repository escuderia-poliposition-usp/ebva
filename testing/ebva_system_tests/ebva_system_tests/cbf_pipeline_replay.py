"""Deterministic in-memory event replay for the three CBF software contracts.

This is not rosbag2, a vehicle profile or an actuator/RC authority test.
"""
from dataclasses import asdict
import argparse
import json
import math
from pathlib import Path
from ebva_cbf_control.models import MODELS
from ebva_cbf_control.software import (CbfSoftwarePipeline, SourceStamp, ScanSample, SpeedSample,
                       SteeringAngleNominal, YawRateNominal)

FRAME = "fixture_control_flu"


def scan(stamp, distance=8., frame=FRAME):
    return ScanSample(SourceStamp(stamp, frame), (distance,) * 181,
                      -math.pi/2, math.pi/180, math.pi/2)


def speed(stamp, value=1., frame=FRAME):
    return SpeedSample(SourceStamp(stamp, frame), value)


def nominal(model, stamp, frame=FRAME):
    cls = SteeringAngleNominal if model == "steering_tangent" else YawRateNominal
    return cls(SourceStamp(stamp, frame), model, .1, .05)


def run_campaign():
    events = []
    def record(model, label, decision, expected):
        if decision.status != expected:
            raise AssertionError((model, label, decision.status, expected))
        if expected != "accepted" and decision.command is not None:
            raise AssertionError("Rejected result leaked a command")
        events.append(dict(model=model, scenario=label, result=asdict(decision)))
    for model in MODELS:
        pipeline = CbfSoftwarePipeline(model)
        assert pipeline.observe_scan(scan(10.), 10.02, 1.)
        assert pipeline.observe_speed(speed(10.01), 10.02, 1.)
        out = pipeline.nominal(nominal(model, 10.02), 10.02, 1.)
        record(model, "open_oldest_header", out, "accepted")
        assert out.command.validity_header == SourceStamp(10., FRAME)
        assert out.command.scan_header.seconds == 10.
        assert out.command.speed_header.seconds == 10.01
        assert out.command.nominal_header.seconds == 10.02
        record(model, "duplicate_nominal", pipeline.nominal(nominal(model, 10.02), 10.02, 1.01),
               "nominal_stamp_rejected")
        # ROS clock stops, independent monotonic lifetime still expires.
        record(model, "paused_clock_scan_expiry", pipeline.nominal(nominal(model, 10.03), 10.02, 1.21),
               "scan_unavailable_or_expired")
        assert pipeline.observe_scan(scan(10.03), 10.03, 1.22)
        record(model, "paused_clock_speed_expiry", pipeline.nominal(nominal(model, 10.04), 10.03, 1.22),
               "speed_unavailable_or_expired")
        assert pipeline.observe_scan(scan(11.), 11.02, 2.)
        assert pipeline.observe_speed(speed(11.01), 11.02, 2.)
        record(model, "fresh_recovery", pipeline.nominal(nominal(model, 11.02), 11.02, 2.), "accepted")
        assert not pipeline.observe_scan(scan(11.03, float("nan")), 11.03, 2.01)
        record(model, "wrong_nominal_frame", pipeline.nominal(nominal(model, 11.04, "other"), 11.04, 2.02),
               "nominal_stamp_rejected")
        assert pipeline.observe_scan(scan(12., .1), 12., 3.)
        assert pipeline.observe_speed(speed(12., 4.), 12., 3.)
        record(model, "provably_infeasible", pipeline.nominal(nominal(model, 12.), 12., 3.),
               "infeasible_box_bound")
        assert pipeline.observe_scan(scan(12.1), 12.1, 3.1)
        assert pipeline.observe_speed(speed(12.1), 12.1, 3.1)
        record(model, "recovery_after_infeasible", pipeline.nominal(nominal(model, 12.1), 12.1, 3.1), "accepted")
        lateral_scan = ScanSample(SourceStamp(12.2, FRAME),
            tuple(.8 if i > 90 else 5. for i in range(181)),
            -math.pi/2, math.pi/180, math.pi/2)
        assert pipeline.observe_scan(lateral_scan, 12.2, 3.2)
        assert pipeline.observe_speed(speed(12.2, 1.2), 12.2, 3.2)
        cls = SteeringAngleNominal if model == "steering_tangent" else YawRateNominal
        expected = "constraint_violation" if model == "yaw_rate" else "infeasible_box_bound"
        record(model, "lateral_solver_rejection", pipeline.nominal(
            cls(SourceStamp(12.2, FRAME), model, .3, .12), 12.2, 3.2), expected)
        assert pipeline.observe_scan(scan(13.), 13.1, 4.)
        assert pipeline.observe_speed(speed(13.09), 13.1, 4.)
        record(model, "fresh_but_incoherent_sources", pipeline.nominal(nominal(model, 13.1), 13.1, 4.), "source_skew")
        record(model, "old_nominal", pipeline.nominal(nominal(model, 13.2), 13.5, 4.4), "nominal_stamp_rejected")
    return dict(kind="pure_dataclass_event_replay", qualified="software_only",
                physical_calibration=False, authority_integrated=False, rosbag2=False,
                events=events, passed=True)


def main(args=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--output", type=Path, help="Optional JSON evidence file")
    options = parser.parse_args(args)
    result = run_campaign()
    serialized = json.dumps(result, indent=2, allow_nan=False) + "\n"
    if options.output:
        options.output.write_text(serialized)
    else:
        print(serialized, end="")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
