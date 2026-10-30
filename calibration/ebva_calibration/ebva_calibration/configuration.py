"""Explicit calibration inputs and output paths, independent of workspace layout."""
from pathlib import Path
import math
import yaml


def load_configuration(path, role):
    if not path: raise ValueError('configuration_file is required; original example is incomplete')
    path=Path(path).expanduser().resolve()
    data=yaml.safe_load(path.read_text())
    if not isinstance(data,dict): raise ValueError('Expected configuration mapping')
    camera=data['camera']; general=data['general']
    if not camera.get('camera_topic'): raise ValueError('camera_topic is required')
    folder_key='data_folder' if role=='pairs' else 'config_folder'
    if not general.get(folder_key): raise ValueError(folder_key+' is required')
    folder=Path(general[folder_key]).expanduser()
    general[folder_key]=str((path.parent/folder).resolve() if not folder.is_absolute() else folder.resolve())
    if role=='pairs':
        if not data['lidar'].get('lidar_topic'): raise ValueError('lidar_topic is required')
        if not math.isfinite(float(general['slop'])) or general['slop']<0: raise ValueError('Invalid slop')
        if int(general['max_file_saved'])<=0: raise ValueError('max_file_saved must be positive')
    elif role=='intrinsic':
        board=camera['chessboard']
        if int(board['rows'])<2 or int(board['columns'])<2 or float(board['square_size_m'])<=0:
            raise ValueError('Specify chessboard rows, columns and square_size_m')
        data['chessboard']=board
        size=camera['image_size']
        if int(size['width'])<=0 or int(size['length'])<=0: raise ValueError('Invalid image size')
        name=Path(general['camera_intrinsic_calibration'])
        if name.name != str(name) or str(name) in ('','.','..'):raise ValueError('Calibration output must be a filename')
    else: raise ValueError('Unknown calibration role')
    return data
