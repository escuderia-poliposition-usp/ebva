"""Validation shared by the native launch profiles and subsystems."""


class ConfigError(RuntimeError):
    """An actionable configuration or dependency failure."""


def boolean(value):
    if isinstance(value, bool):
        return value
    if str(value).lower() in ("true", "false"):
        return str(value).lower() == "true"
    raise ConfigError(f"Booleano invalido: {value!r}; use true ou false")
