#!/usr/bin/env python3
"""Dev shim -> hitbot_imitation.app_reconstruct:main (see that module)."""
import os, sys
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))
from hitbot_imitation.app_reconstruct import main
if __name__ == "__main__":
    raise SystemExit(main())
