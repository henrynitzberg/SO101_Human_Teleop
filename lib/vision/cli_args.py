def add_camera_args(parser):
    """--cam-a/--cam-b device-index args, shared by every script that opens
    a stereo camera pair."""
    parser.add_argument(
        "--cam-a", type=int, default=0, help="Device index for camera A."
    )
    parser.add_argument(
        "--cam-b", type=int, default=1, help="Device index for camera B."
    )
    return parser


def add_calibration_arg(parser, default="calibration/data/stereo_calib.json"):
    """--calibration JSON path arg, shared by every script that triangulates
    points from the stereo pair."""
    parser.add_argument(
        "--calibration",
        type=str,
        default=default,
        help="Stereo calibration JSON from calibration/calibrate_stereo.py.",
    )
    return parser
