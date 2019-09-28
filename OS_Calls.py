from platform import system as system_name  # Returns the system/OS name
from os import system as system_call        # Execute a shell command


def clear_screen():
    """
    Clears the terminal screen.
    """

    # Clear command as function of OS
    command = "cls" if system_name().lower() == "windows" else "clear"

    # Action
    system_call(command)
