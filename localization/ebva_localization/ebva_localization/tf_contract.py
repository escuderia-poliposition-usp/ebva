"""Declared state/TF compatibility and observed TF leases, without ROS I/O."""
from copy import deepcopy
import math

from .state_contract import stamp_nanoseconds

POSE_TYPE = "geometry_msgs/PoseWithCovarianceStamped"
ODOM_TYPE = "nav_msgs/Odometry"
POSE_CAPABILITIES = frozenset(("pose", "pose_covariance"))
TWIST_CAPABILITIES = frozenset(("longitudinal_velocity", "lateral_velocity",
    "vertical_velocity", "roll_rate", "pitch_rate", "yaw_rate", "twist_covariance"))
CAPABILITIES = POSE_CAPABILITIES | TWIST_CAPABILITIES


def _name(value, label, fqn=False):
    if (not isinstance(value, str) or not value or value.strip() != value
            or any(c.isspace() for c in value)):
        raise ValueError(f"{label} must be an explicit name")
    if fqn:
        if not value.startswith("/") or value == "/" or value.endswith("/") or "//" in value:
            raise ValueError(f"{label} must be a fully qualified node name")
    elif value.startswith("/"):
        raise ValueError(f"{label} frame/name must not start with slash")
    return value


def _positive(value, label):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or value <= 0:
        raise ValueError(f"{label} must be finite and positive")
    return value


def _capabilities(value, label):
    if not isinstance(value, list) or not value or any(v not in CAPABILITIES for v in value) or len(set(value)) != len(value):
        raise ValueError(f"{label} must be a nonempty unique list of supported capabilities")
    return set(value)


def validate_profile(profile):
    """Return a copied JSON-ready declaration, or raise ValueError.

    This validates declarations only. It neither discovers ROS nor transforms
    a state message. A declared body on Pose remains a composition assertion.
    """
    if not isinstance(profile, dict):
        raise ValueError("profile must be an object")
    try:
        profile = deepcopy(profile)
        _name(profile["profile_id"], "profile_id")
        local = profile["local_frame"]
        if local is not None:
            _name(local, "local_frame")
        body = _name(profile["body_frame"], "body_frame")
        global_frame = profile["global_frame"]
        if global_frame is not None:
            _name(global_frame, "global_frame")
        if (local is None and global_frame is None) or local == body or (global_frame is not None and global_frame in (local, body)):
            raise ValueError("local/global/body frames must be distinct")
        if not isinstance(profile["require_tf"], bool):
            raise ValueError("require_tf must be boolean")
        if profile["static_lifetime"] != "publisher_present":
            raise ValueError("static_lifetime must explicitly be publisher_present")
        slack = profile["future_tolerance_sec"]
        if (isinstance(slack, bool) or not isinstance(slack, (int, float))
                or not math.isfinite(slack) or slack < 0):
            raise ValueError("future_tolerance_sec must be finite and nonnegative")
        edges = profile["edges"]
        if not isinstance(edges, list):
            raise ValueError("edges must be a list")
        parents = {}
        for edge in edges:
            parent, child = _name(edge["parent"], "edge.parent"), _name(edge["child"], "edge.child")
            _name(edge["publisher"], "edge.publisher", fqn=True)
            if parent == child or child in parents:
                raise ValueError("Each TF child must have one declared parent/authority")
            parents[child] = parent
            if edge["kind"] == "dynamic":
                _positive(edge["max_age_sec"], "edge.max_age_sec")
                if slack >= edge["max_age_sec"]:
                    raise ValueError("Future tolerance must be smaller than each dynamic lease")
            elif edge["kind"] == "static":
                if "max_age_sec" in edge:
                    raise ValueError("Static lifetime is endpoint-based, not a dynamic timeout")
            else:
                raise ValueError("edge.kind must be dynamic or static")
        for child in parents:
            cursor, visited = child, set()
            while cursor in parents:
                if cursor in visited:
                    raise ValueError("Declared TF cycle")
                visited.add(cursor)
                cursor = parents[cursor]

        def has_path(parent, child):
            while child in parents:
                child = parents[child]
                if child == parent:
                    return True
            return False

        if profile["require_tf"]:
            if local is not None:
                if not has_path(local, body):
                    raise ValueError("Missing declared local_frame to body_frame chain")
                if global_frame is not None and not has_path(global_frame, local):
                    raise ValueError("Missing declared global_frame to local_frame chain")
            elif not has_path(global_frame, body):
                raise ValueError("Missing declared global_frame to body_frame chain")
        ports = {}
        for port in profile["state_ports"]:
            name = _name(port["name"], "state_port.name")
            if name in ports:
                raise ValueError("Duplicate state port")
            _name(port["publisher"], "state_port.publisher", fqn=True)
            if not isinstance(port["topic"], str) or not port["topic"] or any(c.isspace() for c in port["topic"]):
                raise ValueError("state_port.topic must be explicit")
            _name(port["frame"], "state_port.frame")
            _name(port["body"], "state_port.body")
            _name(port["model_point"], "state_port.model_point")
            if port["role"] not in ("local", "global"):
                raise ValueError("State port role must be local or global")
            expected_frame = local if port["role"] == "local" else global_frame
            if expected_frame is None or port["frame"] != expected_frame or port["body"] != body:
                raise ValueError("State role/frame/body incompatible with profile")
            if port["stamp"] != "measurement":
                raise ValueError("State ports must preserve measurement time")
            _positive(port["max_age_sec"], "state_port.max_age_sec")
            capabilities = _capabilities(port["provides"], "state_port.provides")
            if port["type"] == ODOM_TYPE:
                if port["twist_frame"] != port["body"]:
                    raise ValueError("Odometry twist must be expressed in its child/body frame")
            elif port["type"] == POSE_TYPE:
                if not capabilities <= POSE_CAPABILITIES or "twist_frame" in port:
                    raise ValueError("Pose cannot offer velocity or a twist frame")
            else:
                raise ValueError("Unsupported state port type")
            ports[name] = port
        consumer_names = set()
        for consumer in profile["consumers"]:
            name = _name(consumer["name"], "consumer.name")
            if name in consumer_names:
                raise ValueError("Duplicate state consumer")
            consumer_names.add(name)
            port = ports.get(consumer["port"])
            if port is None:
                raise ValueError("Unknown consumer state port")
            for key in ("type", "frame", "body", "model_point"):
                if consumer[key] != port[key]:
                    raise ValueError(f"Consumer {key} mismatch; remapping is not a transform")
            needs = _capabilities(consumer["needs"], "consumer.needs")
            if not needs <= set(port["provides"]):
                raise ValueError("State port does not offer all required capabilities")
            _positive(consumer["max_age_sec"], "consumer.max_age_sec")
            if port["max_age_sec"] > consumer["max_age_sec"]:
                raise ValueError("State port age policy exceeds consumer requirement")
        profile["declaration_valid"] = True
        profile["tf_validation_scope"] = "declared_edges" if edges else "not_requested"
        return profile
    except (KeyError, TypeError) as error:
        raise ValueError(f"Malformed state/TF declaration: {error}") from error


class TfAudit:
    """Observational audit of declared TF edges and exact DDS endpoints.

    Graph identity is observed evidence, not authentication. Static transforms
    are accepted only while the exact publishing endpoint remains present.
    """
    def __init__(self, profile):
        self.profile = validate_profile(profile)
        self.edges = {(e["parent"], e["child"]): e for e in self.profile["edges"]}
        self.endpoints = {"dynamic": {}, "static": {}}
        self.observed = {}
        self.last_stamps = {}
        self.rejections = {}
        self.last_ros_ns = None
        self.clock_reset_required = False

    def update_graph(self, endpoints):
        """endpoints[kind][publisher FQN] is a collection of GID strings."""
        self.endpoints = {kind: {name: set(gids) for name, gids in endpoints.get(kind, {}).items()}
                          for kind in ("dynamic", "static")}

    def _graph_issues(self):
        issues = []
        for edge in self.edges.values():
            gids = self.endpoints[edge["kind"]].get(edge["publisher"], set())
            if len(gids) != 1:
                issues.append({"parent": edge["parent"], "child": edge["child"],
                               "reason": "authority_missing" if not gids else "authority_ambiguous"})
        return issues

    def _clock(self, ros_now_ns):
        if not isinstance(ros_now_ns, int) or ros_now_ns <= 0:
            return "clock_unavailable"
        if self.last_ros_ns is not None and ros_now_ns < self.last_ros_ns:
            self.clock_reset_required = True
        self.last_ros_ns = ros_now_ns
        return "clock_reset_required" if self.clock_reset_required else None

    def reject(self, reason):
        self.rejections[reason] = self.rejections.get(reason, 0) + 1
        return reason

    def observe(self, transform, kind, publisher_gid, ros_now_ns, monotonic_ns, received_at=None):
        reason = self._clock(ros_now_ns)
        if reason:
            return self.reject(reason)
        if self._graph_issues():
            return "discovery_not_ready"
        edge_key = (transform.header.frame_id, transform.child_frame_id)
        edge = self.edges.get(edge_key)
        if edge is None:
            return self.reject("undeclared_edge")
        if kind != edge["kind"]:
            return self.reject("tf_kind_mismatch")
        expected = self.endpoints[kind].get(edge["publisher"], set())
        if publisher_gid not in expected:
            return self.reject("unexpected_publisher_gid")
        value = transform.transform
        numbers = (value.translation.x, value.translation.y, value.translation.z,
                   value.rotation.x, value.rotation.y, value.rotation.z, value.rotation.w)
        if not all(math.isfinite(v) for v in numbers):
            return self.reject("nonfinite_transform")
        if abs(math.hypot(*numbers[3:]) - 1.) > 1e-6:
            return self.reject("nonunit_quaternion")
        stamp = transform.header.stamp
        if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
            return self.reject("invalid_stamp")
        if kind == "static":
            # Static transforms may use zero time; endpoint policy controls lifetime.
            stamp_ns = stamp.sec * 1_000_000_000 + stamp.nanosec
            deadline = None
        else:
            try:
                stamp_ns = stamp_nanoseconds(stamp)
            except ValueError:
                return self.reject("invalid_stamp")
            age_ns = ros_now_ns - stamp_ns
            timeout_ns = round(edge["max_age_sec"] * 1e9)
            if age_ns >= timeout_ns:
                return self.reject("transform_expired")
            if -age_ns > round(self.profile["future_tolerance_sec"] * 1e9):
                return self.reject("transform_in_future")
            if edge_key in self.last_stamps and stamp_ns <= self.last_stamps[edge_key]:
                return self.reject("transform_not_newer")
            deadline = monotonic_ns + timeout_ns - max(0, age_ns)
            if received_at is not None:
                received_ros, received_mono = received_at
                deadline = min(deadline, received_mono + timeout_ns
                               - max(0, received_ros - stamp_ns))
                if monotonic_ns >= deadline:
                    return self.reject("transform_expired_before_discovery")
            self.last_stamps[edge_key] = stamp_ns
        self.observed[edge_key] = {"gid": publisher_gid, "stamp_ns": stamp_ns,
                                  "received_ns": monotonic_ns, "deadline_ns": deadline}
        return None

    def report(self, ros_now_ns, monotonic_ns):
        clock_reason = self._clock(ros_now_ns)
        graph_issues = self._graph_issues()
        issues = list(graph_issues)
        if clock_reason:
            issues.append({"reason": clock_reason})
        for key, edge in self.edges.items():
            observation = self.observed.get(key)
            reason = None
            if observation is None:
                reason = "transform_not_observed"
            elif observation["gid"] not in self.endpoints[edge["kind"]].get(edge["publisher"], set()):
                reason = "publishing_endpoint_changed_or_missing"
            elif edge["kind"] == "dynamic":
                if (monotonic_ns < observation["received_ns"] or monotonic_ns >= observation["deadline_ns"]
                        or ros_now_ns-observation["stamp_ns"] >= round(edge["max_age_sec"]*1e9)):
                    reason = "transform_expired"
            if reason:
                issues.append({"parent": key[0], "child": key[1], "reason": reason})
        return {"profile_id": self.profile["profile_id"], "declaration_valid": True,
                "tf_validation_scope": self.profile["tf_validation_scope"],
                "state_validation_scope": "declarations_only",
                "discovery_ready": not graph_issues,
                "observations_valid": not issues and not self.rejections,
                "issues": issues, "rejections": dict(self.rejections),
                "clock_reset_required": self.clock_reset_required,
                "observed_edges": len(self.observed),
                "physical_qualification": False, "authority_authentication": False}
