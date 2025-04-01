# `tiago_sim`

This small package contains ROS2 launch utils tools to ease the deployment of
tiago robot inside gazebo.

[[_TOC_]]

## Install

### Setup

Following the classical ROS2 workflow, create a `<WORKSPACE>` with a `src`
folder accordingly:

```sh
mkdir -p <WORKSPACE>/src
```

Then clone this repo inside the source folder

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

### Dependencies

> [!important]
> The following part indicates how to fetch dependencies inside your source
> folder (`<WORKSPACE>/src`).
> All the commands listed below expect you to be inside `<WORKSPACE>/src`
> (i.e. always do `cd <WORKSPACE>/src` beforehands)

#### `tiago_description`

If the `tiago_description` dependency is not installed on your system (required
to get the tiago's xacro/urdf model description), you can fetch them inside your
workspace with the following instructions.

##### Lightweight (`*_description`)

Theses 'lightweight' dependencies contains only the required `*_description`
folders use to build up tiago's urdf.

> [!note]
> TODO: Find a way to simplify this...

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

##### Full (`*_robots`)

Theses dependencies includes the full `*_robots` github repos, the required
`*_description` and more.

```sh
vcs . < tiago_sim/dependencies.repos
```

#### `linear_feedback_controller`

TODO

```sh
vcs . < tiago_sim/dependencies_lfc.repos
```

### Build

Build using `colcon`:

```sh
source /opt/ros/<DISTRO>/setup.zsh
cd <WORKSPACE>
colcon build --symlink-install
source install/local_setup.zsh
```

## Usage

TODO

```sh
ros2 launch tiago_sim tiago_sim.launch.py resource_path:=$(pwd)/src system_plugin_path:=/opt/ros/jazzy/lib
```

>>> [!tip]
You can always access the full list of arguments of ANY launch file using `-s` option:

```sh
ros2 launch tiago_sim tiago_sim.launch.py -s
```
>>>

### Launch files

This package provides several launch files that can either be used **with or
without** `tiago_description` and its dependencies.

All launch file starting with the `tiago_` prefix use ament lookup functions to
retreive tiago's XACRO from the `tiago_description` package.

#### `tiago_robot_description.launch.py`

TODO

#### `tiago_robot_state_publisher.launch.py`

TODO

#### `tiago_sim.launch.py`

TODO

#### `gz_server.launch.py`

TODO

#### `gz_spawn.launch.py`

TODO

#### `robot_description_from_xacro.launch.py`

TODO

#### `robot_state_publisher_from_xacro.launch.py`

TODO

## Common Issues

### Robot's model is incomplete

When spawning the robot inside Gazebo, you may see that it's 'incomplete',
missing some 3D pieces on the model.

This is due to the fact that, when running the gazebo server, you have to tell
to gazebo the location of 3D models defined wihtin the URDF.

To fix this, you can either:
  - `export GZ_SIM_RESOURCE_PATH=<PATH>:<PATH>:<PATH>` (before launching the GZ server, this is permanent);
  - `GZ_SIM_RESOURCE_PATH=<PATH>:<PATH>:<PATH>; ros2 launch tiago_sim [LAUNCH FILE]`;
  - `ros2 launch tiago_sim [LAUNCH FILE] resource_path:='<PATH>:<PATH>:<PATH>...'`;

When using having only the `*_descption` package inside `<WORKSPACE>/src`, you
can simply set the above mentionned variable the `<WORKSPACE>/src` (e.g. :
`resource_path:=$(pwd)/src` when running `ros2 launch` command from within the
`<WORKSPACE>`).

### ros2_control's `/controller_manager` is not running

If, after launching the simulation, `ros2 node list` doesn't show the
`/controller_manger`, it means that gazebo failed to launch the `ros2_control`
plugin.

You can easily confirm that the plugin is launched by looking at the logs coming
from `[gz_ros_control]` listing all hardware interfaces.

<details>
<summary>

Example

</summary>

```sh
...
[gz-2] [INFO] [1743405688.362940320] [gz_ros_control]: [gz_ros2_control] Fixed joint [wrist_ft_joint] (Entity=109)] is skipped
[gz-2] [INFO] [1743405688.366573870] [gz_ros_control]: Loading controller_manager
[gz-2] [INFO] [1743405688.396108088] [controller_manager]: Subscribing to '/robot_description' topic for robot description.
[gz-2] [WARN] [1743405688.399351176] [gz_ros_control]: Waiting RM to load and initialize hardware...
[gz-2] [INFO] [1743405688.406918690] [controller_manager]: Received robot description from topic.
[gz-2] [INFO] [1743405688.415132795] [gz_ros_control]: The position_proportional_gain has been set to: 0.1
[gz-2] [INFO] [1743405688.415190911] [gz_ros_control]: Loading joint: wheel_right_joint
[gz-2] [INFO] [1743405688.415197916] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415202944] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415209610] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415216775] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415230190] [gz_ros_control]: Loading joint: wheel_left_joint
[gz-2] [INFO] [1743405688.415235381] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415239839] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415246774] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415251119] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415262487] [gz_ros_control]: Loading joint: torso_lift_joint
[gz-2] [INFO] [1743405688.415267546] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415271662] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415276368] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415280681] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415285426] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415289519] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415301743] [gz_ros_control]: Loading joint: arm_1_joint
[gz-2] [INFO] [1743405688.415306609] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415310695] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415315008] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415319246] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415323382] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415328033] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415335729] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415343046] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415352001] [gz_ros_control]: Loading joint: arm_2_joint
[gz-2] [INFO] [1743405688.415356781] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415361016] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415366358] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415370797] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415375186] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415379102] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415385724] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415391284] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415401038] [gz_ros_control]: Loading joint: arm_3_joint
[gz-2] [INFO] [1743405688.415405763] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415409827] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415413882] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415420259] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415424504] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415428585] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415435275] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415443010] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415452141] [gz_ros_control]: Loading joint: arm_4_joint
[gz-2] [INFO] [1743405688.415457092] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415461346] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415465571] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415469974] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415476235] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415480699] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415496660] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415503788] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415514004] [gz_ros_control]: Loading joint: arm_5_joint
[gz-2] [INFO] [1743405688.415519358] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415523598] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415527592] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415531641] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415553348] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415559489] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415566767] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415574414] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415583619] [gz_ros_control]: Loading joint: arm_6_joint
[gz-2] [INFO] [1743405688.415588789] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415593088] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415597218] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415601480] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415605507] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415609754] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415616538] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415621963] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415632832] [gz_ros_control]: Loading joint: arm_7_joint
[gz-2] [INFO] [1743405688.415637506] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415641766] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415645757] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415650005] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415657496] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415661638] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415668183] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415673719] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415684023] [gz_ros_control]: Loading joint: gripper_left_finger_joint
[gz-2] [INFO] [1743405688.415688908] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415693078] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415697153] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415701464] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415706000] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415710020] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415721081] [gz_ros_control]: Loading joint: gripper_right_finger_joint
[gz-2] [INFO] [1743405688.415725857] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415730180] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415734717] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415738849] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415742988] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415746958] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415757250] [gz_ros_control]: Loading joint: head_1_joint
[gz-2] [INFO] [1743405688.415762228] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415766378] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415774275] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415781099] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415785488] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415789547] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415801477] [gz_ros_control]: Loading joint: head_2_joint
[gz-2] [INFO] [1743405688.415806237] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415810374] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415814505] [gz_ros_control]: 		 velocity
[gz-2] [INFO] [1743405688.415818635] [gz_ros_control]: 		 effort
[gz-2] [INFO] [1743405688.415822802] [gz_ros_control]: 	Command:
[gz-2] [INFO] [1743405688.415826817] [gz_ros_control]: 		 position
[gz-2] [INFO] [1743405688.415898333] [gz_ros_control]: Loading sensor: base_imu_sensor
[gz-2] [INFO] [1743405688.415909988] [gz_ros_control]: 	State:
[gz-2] [INFO] [1743405688.415924319] [gz_ros_control]: 		 orientation.x
[gz-2] [INFO] [1743405688.415929605] [gz_ros_control]: 		 orientation.y
[gz-2] [INFO] [1743405688.415934880] [gz_ros_control]: 		 orientation.z
[gz-2] [INFO] [1743405688.415939054] [gz_ros_control]: 		 orientation.w
[gz-2] [INFO] [1743405688.415943260] [gz_ros_control]: 		 angular_velocity.x
[gz-2] [INFO] [1743405688.415947607] [gz_ros_control]: 		 angular_velocity.y
[gz-2] [INFO] [1743405688.415954593] [gz_ros_control]: 		 angular_velocity.z
[gz-2] [INFO] [1743405688.415958951] [gz_ros_control]: 		 linear_acceleration.x
[gz-2] [INFO] [1743405688.415963486] [gz_ros_control]: 		 linear_acceleration.y
[gz-2] [INFO] [1743405688.415969857] [gz_ros_control]: 		 linear_acceleration.z
[gz-2] [INFO] [1743405688.416020912] [gz_ros_control.resource_manager]: Initialize hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416229873] [gz_ros_control.resource_manager]: Successful initialization of hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416543388] [resource_manager]: 'configure' hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416549631] [gz_ros_control]: System Successfully configured!
[gz-2] [INFO] [1743405688.416558012] [resource_manager]: Successful 'configure' of hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416571705] [resource_manager]: 'activate' hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416579214] [resource_manager]: Successful 'activate' of hardware 'ros2_control_tiago_system'
[gz-2] [INFO] [1743405688.416591884] [controller_manager]: Resource Manager has been successfully initialized. Starting Controller Manager services...
...
```

</details>

If this is not the case, the following error message should've appeared:

```sh
...
[gz-2] [Err] [SystemLoader.cc:92] Failed to load system plugin [libgz_ros2_control-system.so] : Could not find shared library.`
...
```

This GZ plugin (`libgz_ros2_control-system.so`) is, most of the time, installed
directly within the `/opt/ros/<DISTRO>/lib` directory, but, for some unknown
reasons, GZ doesn't automatically add this path to the system plugin lookup
path....

To fix this, you can either (replace `<DISTRO>` with `jazzy`, `rolling`, ...,
your current ROS distro):
  - `export GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/<DISTRO>/lib` then call `ros2 launch tiago_sim ...`
  - Call ros2 launch using `GZ_SIM_SYSTEM_PLUGIN_PATH=/opt/ros/<DISTRO>/lib;
    ros2 launch tiago_sim ...`;
  - Call taigo_sim launch files using `ros2 launch tiago_sim
    ... system_plugin_path:=/opt/ros/<DISTRO>/lib'`;
