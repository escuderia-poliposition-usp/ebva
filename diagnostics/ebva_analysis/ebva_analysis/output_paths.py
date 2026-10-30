"""Destinos explícitos e criação exclusiva para ferramentas de análise."""
from pathlib import Path


def output_path(value):
    if not isinstance(value, str) or not value.strip():
        raise ValueError('Defina um arquivo de saída explícito')
    path = Path(value).expanduser().resolve()
    if path.exists():
        raise FileExistsError(str(path))
    if not path.parent.is_dir():
        raise FileNotFoundError('Diretório de saída inexistente: ' + str(path.parent))
    return path
