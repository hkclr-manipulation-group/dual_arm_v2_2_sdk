from setuptools import Distribution, setup


class BinaryDistribution(Distribution):
    def has_ext_modules(self):
        # CMake builds the extension before setuptools packages it.
        return True


setup(distclass=BinaryDistribution)
