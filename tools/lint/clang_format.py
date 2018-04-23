"""Drake's wrapper for the clang-format binary.
"""

import os
import sys


def get_clang_format_path():
    if sys.platform == "darwin":
        path = "/usr/local/bin/clang-format"
    elif os.path.isfile("/usr/bin/clang-format-4.0"):
        path = "/usr/bin/clang-format-4.0"
    else:
        path = "/usr/bin/clang-format"
    if os.path.isfile(path):
        return path
    raise RuntimeError("Could not find required clang-format at " + path)
