import argparse
from localization.aruco_localization import ArucoLocalization
from utils.config import Config

def main():
    parser = argparse.ArgumentParser(description="ArUco marker detection")
    parser.add_argument("--mode", choices=["robot", "board", "all"], default="all", 
                        help="Detection mode: robot (only robot marker), board (only board markers), or all (default)")
    args = parser.parse_args()

    config = Config('config.yaml')
    localization = ArucoLocalization(config)
    localization.run(args.mode)

if __name__ == "__main__":
    main()