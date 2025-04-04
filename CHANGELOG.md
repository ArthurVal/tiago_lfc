# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed

- `CHANGELOG.md`: Markdown indentation issues

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

[unreleased]: https://github.com/ArthurVal/tiago_lfc/compare/v1.0.1...HEAD
[1.0.1]: https://github.com/ArthurVal/tiago_lfc/compare/v1.0.0...v1.0.1
[1.0.0]: https://github.com/ArthurVal/tiago_lfc/releases/tag/v1.0.0
