"""Validate recording requests without ROS, filesystem writes or shell commands."""
from datetime import datetime, timezone
import hashlib
import json
import math
import os
from pathlib import Path
import re
import shutil
import uuid

import yaml

from .resource_publisher import DEFAULT_PATTERNS, METRIC_TOPICS


class ConfigError(RuntimeError):
    """An invalid recording request."""


class UniqueLoader(yaml.SafeLoader):
    pass


def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if key in result:
            raise ConfigError(f"Chave YAML repetida: {key}")
        result[key] = loader.construct_object(value_node, deep=deep)
    return result


UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)


def load_preset(path, duration=None, output_root=None):
    path = Path(path).expanduser().resolve()
    try:
        raw = path.read_bytes()
        data = yaml.load(raw, Loader=UniqueLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ConfigError(f"Nao foi possivel ler {path}: {exc}") from exc
    allowed = {"version", "output_root", "session_prefix", "topics", "duration_s",
               "interval_s", "process_patterns", "min_free_mib", "max_bag_size",
               "max_cache_size", "discovery_timeout_s"}
    if not isinstance(data, dict) or type(data.get("version")) is not int or data["version"] != 2:
        raise ConfigError("Preset precisa definir version: 2")
    if set(data) - allowed:
        raise ConfigError(f"Campos desconhecidos: {sorted(set(data) - allowed)}")
    topics = data.get("topics")
    if not isinstance(topics, list) or not topics:
        raise ConfigError("topics precisa listar nomes absolutos")
    if any(not isinstance(t, str) or not re.fullmatch(r"/(?:[A-Za-z_][A-Za-z0-9_]*/)*[A-Za-z_][A-Za-z0-9_]*", t) for t in topics):
        raise ConfigError("Topico invalido")
    if len(set(topics)) != len(topics):
        raise ConfigError("Topicos repetidos no preset")
    data["topics"] = list(dict.fromkeys([*topics, *METRIC_TOPICS]))
    prefix = data.setdefault("session_prefix", "faixas")
    if not isinstance(prefix, str) or not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_-]{0,63}", prefix):
        raise ConfigError("session_prefix invalido")
    if duration is not None:
        data["duration_s"] = duration
    for key, default, minimum, maximum in (
        ("duration_s", 60.0, 0, 86400), ("interval_s", 1.0, 0.1, 3600),
        ("discovery_timeout_s", 5.0, 0.1, 60),
    ):
        value = data.setdefault(key, default)
        if isinstance(value, bool) or not isinstance(value, (float, int)) or not math.isfinite(value) or not minimum <= value <= maximum:
            raise ConfigError(f"{key} precisa ser numero entre {minimum} e {maximum}")
        data[key] = float(value)
    for key, default, minimum in (("min_free_mib", 2048, 1), ("max_bag_size", 1073741824, 86017), ("max_cache_size", 104857600, 0)):
        value = data.setdefault(key, default)
        if type(value) is not int or value < minimum:
            raise ConfigError(f"{key} precisa ser inteiro >= {minimum}")
    patterns = data.setdefault("process_patterns", DEFAULT_PATTERNS)
    if not isinstance(patterns, list) or not patterns or any(not isinstance(p, str) or not p for p in patterns):
        raise ConfigError("process_patterns precisa listar strings nao vazias")
    destination = output_root if output_root is not None else data.get("output_root", "~/rosbags")
    if not isinstance(destination, (str, Path)) or not str(destination):
        raise ConfigError("output_root precisa definir um caminho")
    destination = Path(destination).expanduser()
    data["output_root"] = (destination if destination.is_absolute() else path.parent / destination).resolve()
    data.update(_path=path, _sha256=hashlib.sha256(raw).hexdigest())
    return data


def output_ancestor(path):
    path = Path(path)
    while not path.exists():
        path = path.parent
    if not path.is_dir():
        raise ConfigError(f"Destino/ancestral nao e diretorio: {path}")
    return path


def available_bytes(path):
    return shutil.disk_usage(output_ancestor(path)).free


def check_storage(config):
    path = config["output_root"]
    ancestor = output_ancestor(path)
    if not os.access(ancestor, os.W_OK | os.X_OK):
        raise ConfigError(f"Destino sem permissao de escrita: {ancestor}")
    free = available_bytes(path)
    if free < config["min_free_mib"] * 1024 ** 2:
        raise ConfigError("Espaco livre abaixo da reserva do preset")
    return free


def recorder_command(config, destination):
    return ["ros2", "bag", "record", "--storage", "sqlite3",
            "--output", str(destination), "--max-bag-size", str(config["max_bag_size"]),
            "--max-cache-size", str(config["max_cache_size"]), *config["topics"]]


def session_metadata(config):
    identifier = config["session_prefix"] + "_" + datetime.now(timezone.utc).strftime("%Y%m%dT%H%M%SZ") + "_" + uuid.uuid4().hex[:8]
    destination = config["output_root"] / identifier
    effective = json.loads(json.dumps(config, default=str))
    return destination, {"id": identifier, "bag_path": str(destination), "preset": effective}
