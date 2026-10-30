"""Resources of campaigns; node parameters are loaded from their owning packages."""
import json
from pathlib import Path
import yaml

def share(package="ebva_system_tests"):
    from ament_index_python.packages import get_package_share_directory
    return Path(get_package_share_directory(package))

def fixture_data(name, root=None):
    root = Path(root) if root else share()
    temporary = root/"data"/(name+".yaml")
    if temporary.is_file():
        return yaml.safe_load(temporary.read_text())
    selected = yaml.safe_load((share()/"profiles"/(name+".yaml")).read_text())
    result = {}
    for node in selected["nodes"]:
        for ref in node["parameters"]:
            path = share(ref["package"])/ref["file"]
            data = yaml.safe_load(path.read_text())
            overlap = set(result)&set(data)
            if overlap:
                raise ValueError("Repeated fixture node configuration: " + str(overlap))
            result.update(data)
    return result

def fixture_path(filename, root=None):
    if root is not None and Path(root).resolve()!=share().resolve() and (Path(root)/"data").is_dir():
        return Path(root)/"data"/filename
    if filename.endswith(".csv"):
        return share("ebva_path_control")/"data/testing"/filename
    return share()/"data"/filename

def fixture_bytes(filename):
    if filename.endswith(".yaml"):
        return yaml.safe_dump(fixture_data(filename[:-5]), sort_keys=False).encode()
    return fixture_path(filename).read_bytes()

def profile_path(name):
    return str(share()/"profiles"/(name+".yaml"))

def configuration(filename, root=None):
    return str(fixture_path(filename, root))

def fixture_exists(filename):
    try:
        fixture_bytes(filename)
        return True
    except (FileNotFoundError,KeyError):
        return False
