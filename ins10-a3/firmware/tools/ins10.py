#!/usr/bin/env python3
"""INS10 AHRS host tool launcher:  python tools/ins10.py --help"""
import os
import sys

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from ins10tool.cli import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
