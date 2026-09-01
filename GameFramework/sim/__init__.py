"""Headless simulation + weight tuning for OvercookedIRL.

Lives inside GameFramework/ on purpose: the game modules use flat imports
(`import actions`, `from config import ...`), so anything that wants to reuse
them has to sit here too. This package holds the OFFLINE tooling -- nothing in
here is imported by the live game.
"""

import os
import sys

# Allow `python -m sim.world` and plain `import sim.world` from anywhere by
# making sure GameFramework/ (the parent) is importable.
_PARENT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if _PARENT not in sys.path:
    sys.path.insert(0, _PARENT)
