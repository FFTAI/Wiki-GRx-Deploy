import argparse
from robot_rcs_gr.sdk.server import RobotServer


# Define the main function with parameters for configuration, frequency, debug interval, and verbosity
def main(config: str, freq: int, debug_interval: int, verbose: bool):
    if not verbose:
        from robot_rcs.logger.fi_logger import Logger

        Logger().state = Logger().STATE_OFF

    robot = RobotServer(freq, debug_interval)
    robot.spin()


if __name__ == "__main__":
    # Use typer to run the main function, parsing command-line arguments
    parser = argparse.ArgumentParser(description='Run server with different configuration')

    # add argument
    parser.add_argument('config', type=str, help = 'configuration file')
    parser.add_argument('--freq', type = int, default=500, help="Main loop frequency in hz. defaults to 400hz.")
    parser.add_argument('--debug_interval', type = int, default=0, help="Debug loop print interval")
    parser.add_argument('--verbose',  action='store_true', help="Print internal debug info")

    args = parser.parse_args()

    # Call the main function with the parsed arguments
    main(args.config, args.freq, args.debug_interval, args.verbose)
