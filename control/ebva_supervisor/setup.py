from pathlib import Path
from setuptools import find_packages, setup
name = "ebva_supervisor"
resources = [
    ("share/ament_index/resource_index/packages", ["resource/" + name]),
    ("share/" + name, ["package.xml", "README.md"]),
]
for folder in ("launch", "config"):
    for parent in sorted({p.parent for p in Path(folder).rglob("*")
                          if p.is_file() and "__pycache__" not in p.parts}):
        resources.append(("share/" + name + "/" + parent.as_posix(),
                          [str(p) for p in sorted(parent.iterdir()) if p.is_file()]))
setup(name=name, version="0.1.0", packages=find_packages(exclude=["test"]),
      data_files=resources, install_requires=["setuptools", "PyYAML"], zip_safe=False,
      maintainer="EBVA", maintainer_email="ebva.usp@gmail.com",
      description="Typed classifier-driven and fixed-method normalized controllers",
      license="MIT", tests_require=["pytest"],
      entry_points={"console_scripts": ["supervisor_node = ebva_supervisor.supervisor_node:main"]})
