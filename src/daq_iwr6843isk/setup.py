from setuptools import find_packages, setup


package_name = "daq_iwr6843isk"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        (
            "share/ament_index/resource_index/packages",
            [f"resource/{package_name}"],
        ),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="hoyoung",
    maintainer_email="hoyoung@todo.todo",
    description="ROS 2 PointCloud2 driver for the TI IWR6843ISK radar.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "pcl_pub = daq_iwr6843isk.pcl_publisher:main",
        ],
    },
)
