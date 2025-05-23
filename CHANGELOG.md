# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

## [1.1.1] - 2025-05-12

### Fixed

- `tiago_robot_description.launch.py`: Generator/list error

## [1.1.0] - 2025-05-12

### Added

- `gz_control()`/`gz_control.launch.py`: New launch file use to control the sim
  (start/pause/...) by cmd line

### Changed

- `gz_tiago_lfc.launch.py`: Now automatically starts the sim after spawning the
  model

### Fixed

- `lfc_parameters.yaml`: Match with the new parameters of  LFC v2.0.0

## [1.0.2] - 2025-04-24

### Fixed

- `CHANGELOG.md`: Markdown indentation issues
- `lfc.repos`: Updates the branches names based on v2.0 of LFC (no more jazzy branch)

## [1.0.1] - 2025-04-04

### Added

- New file: `CHANGELOG.md`

## [1.0.0] - 2025-04-04

### Added

- [`launch`](https://github.com/ArthurVal/tiago_lfc/tree/v1.0.0/launch): All ROS2 launch files:
  - `gz_server.launch.py`
  - `gz_spawn.launch.py`
  - `gz_tiago_lfc.launch.py`
  - `load_controllers.launch.py`
  - `robot_description_from_xacro.launch.py`
  - `robot_state_publisher_from_xacro.launch.py`
  - `switch_controllers.launch.py`
  - `tiago_robot_description.launch.py`
  - `tiago_robot_state_publisher.launch.py`
- [`tiago_lfc`](https://github.com/ArthurVal/tiago_lfc/tree/v1.0.0/tiago_lfc)
  python's package, with modules:
  - [`arguments.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/arguments.py)
    - `all_arguments_from_yaml`
  - [`gz.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/gz.py)
    - `gz_server`
    - `gz_spawn_entity`
  - [`invoke.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/invoke.py)
    - `FunctionSubstitution[T]`
    - `SubstitutionOr[T]`
    - `Invoke[T]`
    - `substitute`
    - `evaluate_[args, kwargs, list, dict]`
  - [`logger.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/logger.py)
    - `logger`
  - [`robot_description.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/robot_description.py)
    - `add_robot_description_from_xacro`
  - [`robot_state_publisher.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/robot_state_publisher.py)
    - `run_robot_state_publisher`
  - [`ros2_control.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/ros2_control.py)
    - `load_controllers`
    - `switch_controllers`
  - [`utils.py`](https://github.com/ArthurVal/tiago_lfc/blob/v1.0.0/tiago_lfc/launch/utils.py)
    - `dict_to_string`
- [`dependencies`](https://github.com/ArthurVal/tiago_lfc/tree/v1.0.0/dependencies)
  folder, with 'vcstools'like dependencies definitions:
  - `tiago_robot.repos`
  - `lfc.repos`
- [`config`](https://github.com/ArthurVal/tiago_lfc/tree/v1.0.0/config)
  (ros2_control controllers' parameters)
  - `lfc_parameters.yaml`
  - `joint_state_broadcaster_parameters.yaml`
- CMakeLists
- LICENSE [BSD-3]
- README
- pacakge.xml

[unreleased]: https://github.com/ArthurVal/tiago_lfc/compare/v1.1.1...HEAD
[1.1.1]: https://github.com/ArthurVal/tiago_lfc/compare/v1.1.0...v1.1.1
[1.1.0]: https://github.com/ArthurVal/tiago_lfc/compare/v1.0.2...v1.1.0
[1.0.2]: https://github.com/ArthurVal/tiago_lfc/compare/v1.0.1...v1.0.2
[1.0.1]: https://github.com/ArthurVal/tiago_lfc/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/ArthurVal/tiago_lfc/releases/tag/v1.0.0
