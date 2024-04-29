# Install script for directory: /home/kaymarrr/drone_ws/src/uav_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/home/kaymarrr/drone_ws/install/uav_description")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/pythonpath.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/pythonpath.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/uav_description-0.0.0-py3.10.egg-info" TYPE DIRECTORY FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_python/uav_description/uav_description.egg-info/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/local/lib/python3.10/dist-packages/uav_description" TYPE DIRECTORY FILES "/home/kaymarrr/drone_ws/src/uav_description/uav_description/" REGEX "/[^/]*\\.pyc$" EXCLUDE REGEX "/\\_\\_pycache\\_\\_$" EXCLUDE)
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  execute_process(
        COMMAND
        "/usr/bin/python3.10" "-m" "compileall"
        "/home/kaymarrr/drone_ws/install/uav_description/local/lib/python3.10/dist-packages/uav_description"
      )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uav_description" TYPE PROGRAM FILES
    "/home/kaymarrr/drone_ws/src/uav_description/scripts/dummy_script.py"
    "/home/kaymarrr/drone_ws/src/uav_description/scripts/trajectory.py"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/uav_description" TYPE EXECUTABLE FILES "/home/kaymarrr/drone_ws/build/uav_description/cpp_node_test")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test"
         OLD_RPATH "/home/kaymarrr/ros2_humble/install/rclcpp/lib:/home/kaymarrr/ros2_humble/install/libstatistics_collector/lib:/home/kaymarrr/ros2_humble/install/rcl/lib:/home/kaymarrr/ros2_humble/install/rmw_implementation/lib:/home/kaymarrr/ros2_humble/install/ament_index_cpp/lib:/home/kaymarrr/ros2_humble/install/rcl_logging_spdlog/lib:/home/kaymarrr/ros2_humble/install/rcl_logging_interface/lib:/home/kaymarrr/ros2_humble/install/rcl_interfaces/lib:/home/kaymarrr/ros2_humble/install/rcl_yaml_param_parser/lib:/home/kaymarrr/ros2_humble/install/libyaml_vendor/lib:/home/kaymarrr/ros2_humble/install/rosgraph_msgs/lib:/home/kaymarrr/ros2_humble/install/statistics_msgs/lib:/home/kaymarrr/ros2_humble/install/builtin_interfaces/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/kaymarrr/ros2_humble/install/rmw/lib:/home/kaymarrr/ros2_humble/install/fastcdr/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/kaymarrr/ros2_humble/install/rosidl_typesupport_c/lib:/home/kaymarrr/ros2_humble/install/rcpputils/lib:/home/kaymarrr/ros2_humble/install/rosidl_runtime_c/lib:/home/kaymarrr/ros2_humble/install/rcutils/lib:/home/kaymarrr/ros2_humble/install/tracetools/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/uav_description/cpp_node_test")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE DIRECTORY FILES
    "/home/kaymarrr/drone_ws/src/uav_description/config"
    "/home/kaymarrr/drone_ws/src/uav_description/meshes"
    "/home/kaymarrr/drone_ws/src/uav_description/robot"
    "/home/kaymarrr/drone_ws/src/uav_description/launch"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/uav_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/uav_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/environment" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_index/share/ament_index/resource_index/packages/uav_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description/cmake" TYPE FILE FILES
    "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_core/uav_descriptionConfig.cmake"
    "/home/kaymarrr/drone_ws/build/uav_description/ament_cmake_core/uav_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/uav_description" TYPE FILE FILES "/home/kaymarrr/drone_ws/src/uav_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/kaymarrr/drone_ws/build/uav_description/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
