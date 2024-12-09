import setuptools

with open("README.md", "r", encoding="utf-8") as f:
    long_description = f.read()

setuptools.setup(
    name="BlenderMavlink",
    version="0.0.1",
    author="Samuel Teague",
    author_email="",
    description="Mavlink interface for controlling blender viewport",
    long_description=long_description,
    long_description_content_type="text/markdown",
    license="Proprietary",
    classifiers=[
        "Programming Language :: Python :: 3",
        "License :: Proprietary",
        "Operating System :: Debian Linux"
    ],
    packages=['BlenderMavlink',
              ],
    include_package_data=True,
    package_data={},
    python_requires=">=3.6",
    install_requires=[
    ],
)
