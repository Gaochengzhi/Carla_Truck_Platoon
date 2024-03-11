from distutils.core import setup
from Cython.Build import cythonize
import numpy

setup(
    ext_modules=cythonize(
        ["cythoncode/cutil.pyx", "cythoncode/controller_baseline.pyx", "cythoncode/router_baseline.pyx"], annotate=True),
    include_dirs=[numpy.get_include()]
)
