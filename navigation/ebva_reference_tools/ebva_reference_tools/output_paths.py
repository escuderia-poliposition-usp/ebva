"""Destino explícito de coleta, sem sobrescrita implícita."""
from pathlib import Path


def output_path(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('Defina o parâmetro output_file com um caminho de saída')
    path = Path(value).expanduser().resolve()
    if path.exists():
        raise FileExistsError(str(path))
    if not path.parent.is_dir():
        raise FileNotFoundError('Diretório de saída inexistente: ' + str(path.parent))
    return path
