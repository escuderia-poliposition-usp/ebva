"""Resolve installed reference data without any bringup/test dependency."""
from pathlib import Path, PurePosixPath
def resolve_reference(value):
    value=str(value)
    if not value.startswith("package://"):return value
    package,separator,relative=value[len("package://"):].partition("/")
    if package!="ebva_path_control" or not separator or PurePosixPath(relative).is_absolute() or ".." in PurePosixPath(relative).parts:
        raise ValueError("Reference must belong to ebva_path_control")
    from ament_index_python.packages import get_package_share_directory
    path=Path(get_package_share_directory(package))/relative
    if not path.is_file():raise ValueError("Installed reference unavailable: "+str(path))
    return str(path)
