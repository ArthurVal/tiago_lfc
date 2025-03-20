cmake_minimum_required(VERSION 3.11)
project(tiago_lfc)

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

ament_python_install_package(${PROJECT_NAME})
ament_auto_package(INSTALL_TO_SHARE launch config)
