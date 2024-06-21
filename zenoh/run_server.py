import typer
from robot_rcs_gr.sdk.server import RobotServer


# Define the main function with parameters for configuration, frequency, debug interval, and verbosity
def main(
    config: str,
    freq: int = typer.Option(500, help="Main loop frequency in hz. defaults to 400hz."),
    debug_interval: int = typer.Option(0, help="Debug loop print interval"),
    verbose: bool = typer.Option(False, help="Print internal debug info"),
):
    if not verbose:
        from robot_rcs.logger.fi_logger import Logger

        Logger().state = Logger().STATE_OFF

    # Initialize the RobotServer and run it withwith the given frequency and debug interval.
    robot = RobotServer(freq, debug_interval)
    robot.spin()


if __name__ == "__main__":
    # Use typer to run the main function, parsing command-line arguments
    typer.run(main)
