# tiago_sim

TODO

## Install

### Setup

Following the classical ROS2 workflow, create a `<WORKSPACE>` accordingly:

```sh
mkdir -p <WORKSPACE>/src
```

The clone this repo inside the source folder

```sh
cd <WORKSPACE>/src
git clone git clone https://gitlab.laas.fr/avaliente/tiago_sim.git
```

The workspace should look something like this (named `tiago_sim_ws` in this example):

```sh
tree tiago_sim_ws -L 3
tiago_sim_ws
└── src
    └── tiago_sim
        ├── CMakeLists.txt
        ├── config
        ├── dependencies_lfc.repos
        ├── dependencies.repos
        ├── launch
        ├── LICENSE
        ├── package.xml
        ├── README.md
        └── tiago_sim

6 directories, 6 files
```

### Dependencies - `tiago_description`

If the `tiago_description` dependency is not installed on your system (required
to get the tiago's xacro/urdf model description), you can fetch them inside your
workspace following theses instructions.

Inside `<WORKSPACE>/src` you can either:

- Fetch the 'lightweight' version of the dependencies, containing only
  `*_description` folders:

```sh
wget -O - https://github.com/Tiago-Harmonic/tiago_robot/archive/jazzy.tar.gz | tar -xz --strip=1 tiago_robot-jazzy/tiago_description
wget -O - https://github.com/Tiago-Harmonic/omni_base_robot/archive/jazzy.tar.gz | tar -xz --strip=1 omni_base_robot-jazzy/omni_base_description
wget -O - https://github.com/Tiago-Harmonic/pmb2_robot/archive/jazzy.tar.gz | tar -xz --strip=1 pmb2_robot-jazzy/pmb2_description
wget -O - https://github.com/Tiago-Harmonic/pal_robotiq_gripper/archive/jazzy.tar.gz | tar -xz --strip=1 pal_robotiq_gripper-jazzy/pal_robotiq_description
wget -O - https://github.com/Tiago-Harmonic/pal_hey5/archive/jazzy.tar.gz | tar -xz --strip=1 pal_hey5-jazzy/pal_hey5_description
wget -O - https://github.com/Tiago-Harmonic/pal_gripper/archive/jazzy.tar.gz | tar -xz --strip=1 pal_gripper-jazzy/pal_gripper_description
git clone https://github.com/Tiago-Harmonic/launch_pal.git
git clone https://github.com/Tiago-Harmonic/pal_urdf_utils.git --branch jazzy
```

- Get the full dependencies from github repos using:

```sh
vcs . < tiago_sim/dependencies.repos
```

### Dependencies - `linear_feedback_controller`

TODO

```sh
vcs . < tiago_sim/dependencies_lfc.repos
```

### Build

TODO

```sh
source /opt/ros/<DISTRO>/setup.zsh
cd <WORKSPACE>
colcon build --symlink-install
source install/local_setup.zsh
```

## Usage

TODO

```sh
ros2 launch tiago_sim tiago_sim.launch.py [...]
```

```sh
ros2 launch tiago_sim tiago_sim.launch.py -s
```

### Launch files

TODO

#### `gz_server.launch.py`

TODO

#### `gz_spawn.launch.py`

TODO

#### `robot_description_from_xacro.launch.py`

TODO

#### `robot_state_publisher_from_xacro.launch.py`

TODO

#### `tiago_robot_description.launch.py`

TODO

#### `tiago_robot_state_publisher.launch.py`

TODO

#### `tiago_sim.launch.py`

TODO
