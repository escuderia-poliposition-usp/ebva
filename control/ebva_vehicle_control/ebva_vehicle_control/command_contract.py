"""Static command-route audit for launch composition; never opens a device.

Routes are declarations checked against the composition by package tests.
A successful audit is not proof of the live ROS graph or runtime watchdogs.
"""
from dataclasses import asdict, dataclass
import math
from typing import Optional, Tuple

ConfigError = ValueError

FINAL_TOPICS = ("/commands/motor/speed", "/commands/servo/position")
NEUTRAL_POLICY = "resume_autonomy_if_all_inputs_valid"


@dataclass(frozen=True)
class CommandRoute:
    """One declared path to a final publisher, including its RC authority."""

    name: str
    chain: Tuple[str, ...]
    outputs: Tuple[str, ...] = FINAL_TOPICS
    rc_authority: Optional[str] = None
    rc_priority: bool = False
    rc_timeout_s: Optional[float] = None
    autonomy_timeout_s: Optional[float] = None
    rc_loss: str = "unspecified"
    autonomy_loss: str = "unspecified"
    neutral_policy: str = "unspecified"


def _positive_timeout(value):
    return (isinstance(value, (int, float)) and not isinstance(value, bool)
            and math.isfinite(value) and value > 0)


def audit_command_routes(routes, *, actuation_required):
    """Report declared ownership, RC coverage, expiry and neutral policy.

    Callers supply resolved absolute topics/nodes. This v1 audit covers the
    VESC speed/servo backend only; a different backend needs its own contract.
    """
    routes = tuple(routes)
    issues = []
    writers = {topic: set() for topic in FINAL_TOPICS}

    def issue(code, detail):
        issues.append({"code": code, "detail": detail})

    if not actuation_required and routes:
        issue("unexpected_actuation", "Perfil sem atuacao declara rota ate os atuadores")
    for route in routes:
        if (not route.chain or not all(isinstance(node, str) and node.startswith("/")
                                      and node != "/" for node in route.chain)):
            issue("invalid_route", f"{route.name}: cadeia precisa de nomes absolutos")
            continue
        if not route.outputs or len(set(route.outputs)) != len(route.outputs):
            issue("invalid_outputs", f"{route.name}: saidas vazias ou repetidas")
        for topic in route.outputs:
            if topic not in writers:
                issue("unsupported_output", f"{route.name}: backend nao coberto: {topic}")
            else:
                writers[topic].add(route.chain[-1])
        if route.rc_authority not in route.chain or route.rc_priority is not True:
            issue("rc_bypass", f"{route.name}: rota nao declara prioridade RC em sua cadeia")
        if not _positive_timeout(route.rc_timeout_s) or route.rc_loss != "stop":
            issue("rc_expiry_unspecified", f"{route.name}: falta prazo finito e parada por perda de RC")
        if not _positive_timeout(route.autonomy_timeout_s) or route.autonomy_loss != "stop":
            issue("autonomy_expiry_unspecified", f"{route.name}: falta prazo finito e parada por perda de autonomia")
        if route.neutral_policy != NEUTRAL_POLICY:
            issue("neutral_policy_mismatch", f"{route.name}: retorno ao neutro deve exigir entradas validas")

    if actuation_required:
        for topic, owners in writers.items():
            if not owners:
                issue("missing_writer", f"{topic}: nenhum escritor final declarado")
            elif len(owners) > 1:
                issue("competing_writers", f"{topic}: " + ", ".join(sorted(owners)))
        motor, servo = (writers[topic] for topic in FINAL_TOPICS)
        if len(motor) == len(servo) == 1 and motor != servo:
            issue("split_command_owner", "Motor e direcao pertencem a escritores finais diferentes")

    return {
        "version": 1,
        "scope": "declared_launch_routes_not_live_graph",
        "compatible": not issues,
        "status": "incompatible" if issues else ("declared" if actuation_required else "no_actuation"),
        "writers": {topic: sorted(owners) for topic, owners in writers.items()},
        "routes": [asdict(route) for route in routes],
        "issues": issues,
    }


def observed_command_routes(components):
    """Model the current subsystems.vehicle/application sources, not a new mux."""
    routes = []
    if "vehicle" in components:
        routes.append(CommandRoute(
            "ackermann_backend",
            ("/ackermann_mux", "/ackermann_to_vesc_node"),
        ))
    if "application" in components:
        # Local override exists, but the serial/auto values do not expire.
        routes.append(CommandRoute(
            "legacy_rc_bridge", ("/rc_reader_node",),
            rc_authority="/rc_reader_node", rc_priority=True,
            neutral_policy=NEUTRAL_POLICY,
        ))
    return routes


def require_command_contract(report):
    """Reject an unqualified declaration before returning hardware actions."""
    if not report or not report.get("compatible"):
        issues = (report or {}).get("issues", [])
        details = "; ".join(item["code"] + ": " + item["detail"] for item in issues)
        raise ConfigError(
            "Contrato de comando incompativel; execucao bloqueada antes dos drivers. "
            + (details or "Auditoria de comando ausente.")
            + " Use check_only:=true offline:=true para inspecionar."
        )
