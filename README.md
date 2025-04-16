## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `teste_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).

## -> Caso mudar o URDF - OK
xacro path/to/model/model.xacro > path/to/model/model.urdf
colcon build --symlink-install

## -> Carregar modelo do robo - OK

ros2 launch teste_bot rsp.launch.py use_sim_time:=true

## -> Carregar o gazebo - OK
gz sim path/to/model/model.sdf

## -> Habilitar ponte entre gazebo e ros2 - OK
ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:=$WORKSPACE/ros_gz/ros_gz_bridge/test/config/full.yaml

## -> Dar spawm no robo
ros2 launch ros_gz_sim gz_spawm_model.launch.py file:=path/to/model/model.urdf entity_name:=my_robot topic:=robot_description z:=0.05

## -> Carregar teleop
ros2 run teleop_twist_keyboard teleop_twist_keyboard