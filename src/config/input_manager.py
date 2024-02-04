import argparse


def config_args():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "-d",
        "--debug",  # you can use it to change world settings
        help="Enable debug mode and change the conifg data to load on a small map",
    )
    parser.add_argument(
        "-dr",
        "--draw",
        action="append",  # you can use -dr multiple times
        help="Enable draw mode and draw the plot of the obstacle sensor",
    )
    parser.add_argument(
        "-t",
        "--town",
        type=str,
        help="Choose the town to load",
    )
    parser.add_argument(
        "-l",
        "--log",
        type=str,
        default="both",
        choices=["file", "console", "all", "none", ""],
        help="Enable log",
    )
    args = parser.parse_args()
    return args
