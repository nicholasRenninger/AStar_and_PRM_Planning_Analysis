# Returns the system/OS name
from platform import system as system_name 

# Execute a shell command
from os import system as system_call


def clear_screen():
    """
    Clears the terminal screen.
    """

    # Clear command as function of OS
    command = "cls" if system_name().lower() == "windows" else "clear"

    # Action
    system_call(command)
