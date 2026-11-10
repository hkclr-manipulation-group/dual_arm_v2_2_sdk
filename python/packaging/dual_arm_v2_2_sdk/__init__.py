"""Python package wrapper around the pybind11 ControlApi extension.

The extension file keeps the module init name ``dual_arm_v2_2_sdk``. Loading it
as ``dual_arm_v2_2_sdk.dual_arm_v2_2_sdk`` matches that init function, and
``$ORIGIN/libs`` still points at this package directory.
"""

import importlib.util
import sys
from pathlib import Path

_here = Path(__file__).resolve().parent
_candidates = sorted(_here.glob("dual_arm_v2_2_sdk.cpython-*.so"))
if not _candidates:
    raise ImportError(f"dual_arm_v2_2_sdk extension not found in {_here}")

_mod_name = "dual_arm_v2_2_sdk.dual_arm_v2_2_sdk"
_spec = importlib.util.spec_from_file_location(_mod_name, _candidates[-1])
if _spec is None or _spec.loader is None:
    raise ImportError(f"cannot load {_candidates[-1]}")

_native = importlib.util.module_from_spec(_spec)
sys.modules[_mod_name] = _native
_spec.loader.exec_module(_native)

__version__ = _native.__version__
__doc__ = _native.__doc__

for _name in dir(_native):
    if _name.startswith("_"):
        continue
    globals()[_name] = getattr(_native, _name)

__all__ = [name for name in dir(_native) if not name.startswith("_")]
