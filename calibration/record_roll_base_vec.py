import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))

from lib.calibration.record_rotation_vector import record_rotation_vector
from lib.calibration.record_two_samples import add_common_args


def main():
    parser = argparse.ArgumentParser(
        description="Record the 'roll' rotation vector (see lib/calibration/record_rotation_vector.py)."
    )
    add_common_args(parser)
    args = parser.parse_args()
    record_rotation_vector("roll", args)


if __name__ == "__main__":
    main()
