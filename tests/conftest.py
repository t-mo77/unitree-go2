"""Put the project root (/work in the container) and nl_control/ at the front of sys.path."""

import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent.parent

for path in (ROOT / "nl_control", ROOT):
    if str(path) not in sys.path:
        sys.path.insert(0, str(path))
