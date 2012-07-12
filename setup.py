import sys
import os

from setuptools import setup

def get_version():
    """
    Gets the version number. Pulls it from the source files rather than
    duplicating it.
    
    """
    # we read the file instead of importing it as root sometimes does not
    # have the cwd as part of the PYTHONPATH
    fn = os.path.join(os.path.dirname(__file__), 'src', 'splitarea', '__init__.py')
    
    try:
        lines = open(fn, 'r').readlines()
    except IOError:
        raise RuntimeError("Could not determine version number"
                           "(%s not there)" % (fn))
    version = None
    for l in lines:
        # include the ' =' as __version__ might be a part of __all__
        if l.startswith('__version__ =', ):
            version = eval(l[13:])
            break
    if version is None:
        raise RuntimeError("Could not determine version number: "
                           "'__version__ =' string not found")
    return version

setup(
    name = "splitarea",
    version = get_version(),
    author = "Martijn Meijers",
    author_email = "b dot m dot meijers at tudelft dot nl",
    license = "MIT license",
    description = "SplitArea: an algorithm for splitting polygons in a topological data structure",
    url = "",
    package_dir = {'':'src'},
)