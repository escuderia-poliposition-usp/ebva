"""Verify package independence without running nodes."""
import ast
from pathlib import Path
import unittest
import xml.etree.ElementTree as ET


class PackageIndependenceTests(unittest.TestCase):

    def test_runtime_does_not_import_other_checkouts(self):
        root = Path(__file__).resolve().parents[1]
        for path in (root / "ebva_diagnostics").glob("*.py"):
            for node in ast.walk(ast.parse(path.read_bytes())):
                imported = [alias.name for alias in node.names] if isinstance(node, ast.Import) else [node.module or ""] if isinstance(node, ast.ImportFrom) else []
                self.assertFalse(any(name.split(".")[0] in {"bench", "tools", "experiments", "ebva_bringup"} for name in imported))
        manifest = ET.parse(root / "package.xml").getroot()
        self.assertEqual(manifest.findtext("name"), "ebva_diagnostics")
        self.assertTrue((root / "resource/ebva_diagnostics").is_file())
