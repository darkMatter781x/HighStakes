# Makes sure that the project is fully built before running the visualization

import subprocess


def build(target: str,
          exit_on_error: bool = True,
          silent: bool = None,
          build_command: list[str] = None,
          make_opts: list[str] = []):
    """
    Build the project
    target: "sim" or "bot"
    """

    def create_build_command():
        command = ["make"]
        match target:
            case "sim":
                command.append("VEXIDE_SIMULATION=1")
                command.append("quick")
            case "bot":
                pass
            case _:
                raise ("Invalid target")
        command.append("-j")
        command.append("$(nproc)")
        return command

    if build_command is None:
        command = create_build_command()
    else:
        command = build_command
    command.extend(make_opts)
    command_str = " ".join(command)
    print("running command: ", command_str)
    process = subprocess.run(command_str, shell=True, capture_output=silent, check=True)
    if process.returncode != 0:
        if exit_on_error:
            print("Error building monolith, exiting...")
            exit(1)
        else:
            raise ("Error building monolith")
