import struct
from types import SimpleNamespace

import numpy as np
import pytest

from ebva_analysis.map_to_pdf import read_pcd_xyz, render
from ebva_analysis.output_paths import output_path


def test_ascii_and_binary_pcd_have_the_same_geometry(tmp_path):
    header = "VERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\nWIDTH 2\nHEIGHT 1\nPOINTS 2\n"
    ascii_file, binary_file = tmp_path / "ascii.pcd", tmp_path / "binary.pcd"
    ascii_file.write_text(header + "DATA ascii\n1 2 0\n3 4 0\n")
    binary_file.write_bytes((header + "DATA binary\n").encode() + struct.pack("<6f", 1, 2, 0, 3, 4, 0))
    expected = np.array([[1., 2., 0.], [3., 4., 0.]])
    np.testing.assert_allclose(read_pcd_xyz(ascii_file), expected)
    np.testing.assert_allclose(read_pcd_xyz(binary_file), expected)


def test_pdf_render_does_not_overwrite_existing_output(tmp_path):
    output = tmp_path / "map.pdf"
    args = SimpleNamespace(style="scatter", point_size=1., resolution=.05, density=False,
                           title="Mapa", input="points.pcd", output=str(output))
    render(np.array([[0., 0., 0.], [1., 1., 0.]]), args)
    first = output.read_bytes()
    assert first.startswith(b"%PDF")
    with pytest.raises(FileExistsError):
        render(np.array([[10., 10., 0.]]), args)
    assert output.read_bytes() == first


def test_output_path_rejects_implicit_or_existing_targets(tmp_path):
    with pytest.raises(ValueError):
        output_path("")
    existing = tmp_path / "data.csv"
    existing.write_text("original")
    with pytest.raises(FileExistsError):
        output_path(str(existing))
    assert output_path(str(tmp_path / "new.csv")) == tmp_path / "new.csv"


def test_ascii_multicount_descriptor_does_not_shift_xyz(tmp_path):
    header = ("VERSION 0.7\nFIELDS descriptor x y z\nSIZE 4 4 4 4\n"
              "TYPE F F F F\nCOUNT 2 1 1 1\nWIDTH 1\nHEIGHT 1\nPOINTS 1\n")
    ascii_file, binary_file = tmp_path / "descriptor-ascii.pcd", tmp_path / "descriptor-binary.pcd"
    ascii_file.write_text(header + "DATA ascii\n9 8 1 2 3\n")
    binary_file.write_bytes((header + "DATA binary\n").encode() + struct.pack("<5f", 9, 8, 1, 2, 3))
    np.testing.assert_allclose(read_pcd_xyz(ascii_file), [[1., 2., 3.]])
    np.testing.assert_allclose(read_pcd_xyz(binary_file), [[1., 2., 3.]])
