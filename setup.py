from setuptools import setup, find_packages

setup(
    name='vzense',
    version='0.1.0',
    packages=find_packages(include=['vzense']),
    package_dir={"":"src"}
)