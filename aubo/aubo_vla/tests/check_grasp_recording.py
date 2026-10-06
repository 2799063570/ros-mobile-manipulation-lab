#!/usr/bin/python3
"""Compatibility entry for the dedicated Gazebo grasp workflow."""
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'src'))
from aubo_vla.grasp_workflow import main, task_state, verify_placements

if __name__ == '__main__':
    sys.exit(main())
