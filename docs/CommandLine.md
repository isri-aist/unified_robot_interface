# Command line

URI is a single executable, `uri`, with one subcommand per tool. Run
`uri --help` or `uri <command> --help` to see these options in the terminal.

| Command | Runs on | Purpose |
|---|---|---|
| [`uri manager`](#uri-manager) | control PC | Runs the controller (mc_rtc) for the whole fleet |
| [`uri interface`](#uri-interface) | robot PC, or control PC | Robot-side process of one robot; loads its driver plugin |
| [`uri viewer`](#uri-viewer) | anywhere with the driver | Loads a driver directly and prints its joint state (no manager, no network) |
| [`uri create_new_driver`](#uri-create-new-driver) | dev machine | Scaffolds a new driver project |

## uri manager

```bash
uri manager -c <mc_rtc.yaml>
```

| Option / variable | Meaning |
|---|---|
| `-c, --config <file>` | mc_rtc configuration, **required** |
| `URI_DRIVER_PATH=<dir>[:<dir>...]` | Extra directories searched for driver plugins by `uri interface` and `uri viewer`, before the install prefix (inherited by autostarted interfaces) |
| `MC_RT_FREQ=<ms>` | Period of the `SCHED_DEADLINE` reservation for the main thread, in milliseconds (default 1) |

See [Robot manager](docRobotManager.html) for the configuration file.

## uri interface

```bash
uri interface -c <config.yaml> [-n <name>]
```

| Option | Meaning |
|---|---|
| `-c, --config <file>` | Robot interface configuration, **required** |
| `-n, --name <name>` | Robot name; overrides `name:` in the file. Must match the robot's key under `Robots:` in the manager's `mc_rtc.yaml`. |

Drivers are searched in the directories of the `URI_DRIVER_PATH` environment
variable (colon-separated), then in URI's install prefix.

Start it before `uri manager`, or let the manager start it with
`robot_interface.autostart: true`. See [Robot interface](docRobotInterface.html).

## uri viewer

```bash
uri viewer -c <config.yaml> [-n <name>] [-r <rate>]
```

| Option | Meaning |
|---|---|
| `-c, --config <file>` | Robot interface configuration, or the manager's `mc_rtc.yaml`, **required** |
| `-n, --name <name>` | Robot to view under `Robots:`, when `--config` is an `mc_rtc.yaml` with several robots |
| `-r, --rate <hz>` | Display refresh rate (default 10) |

It only reads the state and never sends commands. It finds drivers the same way
as `uri interface`, including `URI_DRIVER_PATH`.

## uri create_new_driver

```bash
uri create_new_driver <name> [folder] [--no-git]
```

| Argument / option | Meaning |
|---|---|
| `name` | Driver name in PascalCase, e.g. `Franka`. Generates `franka_driver/` with the class `RobotDriverFranka`. **Required** |
| `folder` | Directory in which to create the project (default: current directory) |
| `--no-git` | Don't initialize a git repository in the project |

See [Create a new robot driver](docNewRobotDriver.html).
