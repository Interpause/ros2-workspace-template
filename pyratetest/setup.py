from setuptools import setup
from glob import glob

package_name = "pyratetest"

setup(
    name=package_name,
    version="0.0.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name, glob("launch/*.launch.py")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="John-Henry Lim",
    maintainer_email="42513874+Interpause@users.noreply.github.com",
    description="Example package that determines max communication rate.",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "pub = pyratetest.publish:main",
            "sub = pyratetest.subscribe:main",
        ]
    },
)
