"""Declarative orchestration. Inspection never imports a component launch."""
import os
import re
from pathlib import Path, PurePosixPath
import yaml

class ProfileError(ValueError):
    pass

class UniqueLoader(yaml.SafeLoader):
    pass

def _mapping(loader, node, deep=False):
    result = {}
    for key_node, value_node in node.value:
        key = loader.construct_object(key_node, deep=deep)
        if not isinstance(key, str) or key in result:
            raise ProfileError("Repeated or non-string YAML key: " + str(key))
        result[key] = loader.construct_object(value_node, deep=deep)
    return result
UniqueLoader.add_constructor(yaml.resolver.BaseResolver.DEFAULT_MAPPING_TAG, _mapping)

def _fields(value, allowed, required, label):
    if not isinstance(value, dict):
        raise ProfileError(label + " must be a mapping")
    unknown, missing = set(value)-set(allowed), set(required)-set(value)
    if unknown or missing:
        raise ProfileError(f"{label}: unknown fields {sorted(unknown)}, missing {sorted(missing)}")
    return value

def _text(value, label):
    if not isinstance(value, str) or not value.strip():
        raise ProfileError(label + " must be a nonempty string")
    return value

def _package(value):
    value = _text(value, "package")
    if not re.fullmatch(r"[a-z][a-z0-9_]*", value):
        raise ProfileError("Invalid ROS package: " + value)
    return value

def _resource(package, filename, get_share):
    package = _package(package)
    filename = _text(filename, "file")
    relative = PurePosixPath(filename)
    if relative.is_absolute() or ".." in relative.parts or "\\" in filename or "\0" in filename:
        raise ProfileError("Resource must be relative to its installed package: " + filename)
    try:
        path = Path(get_share(package)).joinpath(*relative.parts)
    except Exception as exc:
        raise ProfileError("Package unavailable: " + package) from exc
    # Installed ament resources may be symlinks into the source/build tree.
    if not path.is_file():
        raise ProfileError("Resource unavailable: " + str(path))
    return str(path)

def _reference(value, get_share):
    _fields(value, {"package", "file"}, {"package", "file"}, "configuration reference")
    if value["package"] == "ebva_bringup":
        raise ProfileError("Component configuration cannot belong to ebva_bringup")
    if Path(_text(value["file"], "configuration file")).suffix not in (".yaml", ".yml", ".json"):
        raise ProfileError("A configuration reference must name YAML or JSON")
    return _resource(value["package"], value["file"], get_share)

def inspect_profile(profile="manual", *, get_share=None, get_prefix=None):
    if get_share is None:
        from ament_index_python.packages import get_package_share_directory
        get_share = get_package_share_directory
    if get_prefix is None:
        from ament_index_python.packages import get_package_prefix
        get_prefix = get_package_prefix
    selected = Path(_text(str(profile), "profile")).expanduser()
    if selected.is_absolute():
        path = selected
    else:
        name = str(selected)
        if not re.fullmatch(r"[A-Za-z0-9_]+(?:\.yaml)?", name):
            raise ProfileError("Use an installed profile name or an absolute YAML path")
        path = Path(get_share("ebva_bringup"))/"profiles"/(name if name.endswith(".yaml") else name+".yaml")
    try:
        data = yaml.load(path.read_text(), Loader=UniqueLoader)
    except (OSError, yaml.YAMLError) as exc:
        raise ProfileError("Cannot load profile " + str(path) + ": " + str(exc)) from exc
    _fields(data, {"version", "name", "namespace", "launches", "nodes"}, {"version", "name"}, "profile")
    if type(data["version"]) is not int or data["version"] != 1:
        raise ProfileError("Unsupported profile version; expected integer 1")
    name = _text(data["name"], "profile name")
    namespace = data.get("namespace", "")
    if not isinstance(namespace, str) or not re.fullmatch(r"/?(?:[A-Za-z_][A-Za-z0-9_]*(?:/[A-Za-z_][A-Za-z0-9_]*)*)?/?", namespace):
        raise ProfileError("Invalid profile namespace")
    namespace = "/" + namespace.strip("/") if namespace.strip("/") else "/"
    report = {"version": 1, "name": name, "namespace": namespace, "profile_file": str(path),
              "launches": [], "nodes": [], "scope": "static_resources_only"}
    seen = set()
    for kind in ("launches", "nodes"):
        entries = data.get(kind, [])
        if not isinstance(entries, list):
            raise ProfileError(kind + " must be a list")
        for entry in entries:
            allowed = {"id", "package", "file", "arguments"} if kind == "launches" else {"id", "package", "executable", "name", "parameters"}
            required = {"id", "package", "file"} if kind == "launches" else {"id", "package", "executable"}
            _fields(entry, allowed, required, kind + " entry")
            identity = _text(entry["id"], "component id")
            if identity in seen:
                raise ProfileError("Duplicate component id: " + identity)
            seen.add(identity)
            package = _package(entry["package"])
            item = dict(entry)
            if kind == "launches":
                item["launch_file"] = _resource(package, entry["file"], get_share)
                arguments = entry.get("arguments", {})
                if not isinstance(arguments, dict):
                    raise ProfileError("Launch arguments must map names to configuration references")
                item["arguments"] = {}
                for key, value in arguments.items():
                    key = _text(key, "launch argument")
                    item["arguments"][key] = _reference(value, get_share)
                item["configuration"] = "explicit references" if arguments else "default do pacote"
            else:
                executable = _text(entry["executable"], "executable")
                if not re.fullmatch(r"[A-Za-z0-9_][A-Za-z0-9_.-]*", executable):
                    raise ProfileError("Invalid executable name")
                try:
                    target = Path(get_prefix(package))/"lib"/package/executable
                except Exception as exc:
                    raise ProfileError("Package unavailable: " + package) from exc
                if not target.is_file() or not os.access(target, os.X_OK):
                    raise ProfileError("Executable unavailable: " + str(target))
                if "name" in entry and not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]*", _text(entry["name"], "node name")):
                    raise ProfileError("Invalid node name")
                references = entry.get("parameters", [])
                if not isinstance(references, list):
                    raise ProfileError("Node parameters must be a list of file references")
                item["parameters"] = [_reference(value, get_share) for value in references]
                if any(Path(value).suffix not in (".yaml", ".yml") for value in item["parameters"]):
                    raise ProfileError("Direct ROS node parameters require YAML")
                item["executable_file"] = str(target)
                item["configuration"] = "explicit references" if references else "defaults declared by node"
            report[kind].append(item)
    if not seen:
        raise ProfileError("Profile must select at least one component")
    direct_names = [node.get("name", node["executable"]) for node in report["nodes"]]
    if len(direct_names) != len(set(direct_names)):
        raise ProfileError("Duplicate direct node name in the selected namespace")
    return report

def actions_for(report):
    from launch.actions import GroupAction, IncludeLaunchDescription, RegisterEventHandler, EmitEvent
    from launch.event_handlers import OnProcessExit
    from launch.events import Shutdown
    from launch.launch_description_sources import AnyLaunchDescriptionSource
    from launch_ros.actions import Node, PushRosNamespace
    actions = [PushRosNamespace(report["namespace"])]
    for item in report["launches"]:
        actions.append(IncludeLaunchDescription(AnyLaunchDescriptionSource(item["launch_file"]),
                                               launch_arguments=item["arguments"].items()))
    for item in report["nodes"]:
        options = {"package": item["package"], "executable": item["executable"],
                   "parameters": item["parameters"], "output": "screen"}
        if "name" in item:
            options["name"] = item["name"]
        node = Node(**options)
        actions.append(RegisterEventHandler(OnProcessExit(target_action=node, on_exit=[
            EmitEvent(event=Shutdown(reason="Direct profile node exited: " + item["id"]))])))
        actions.append(node)
    return [GroupAction(actions=actions)]

def launch_system(context):
    import json
    from launch.actions import LogInfo
    values = context.launch_configurations
    check = values.get("check_only", "false")
    if check not in ("true", "false"):
        raise ProfileError("check_only must be true or false")
    report = inspect_profile(values.get("profile", "manual"))
    if check == "true":
        return [LogInfo(msg=json.dumps(report, ensure_ascii=False))]
    from .launching import claim_stack, running_conflicts
    names = [report["namespace"].rstrip("/") + "/" + node.get("name", node["executable"])
             for node in report["nodes"]]
    if names:
        running_conflicts(names)
    claim_stack()
    return actions_for(report)
