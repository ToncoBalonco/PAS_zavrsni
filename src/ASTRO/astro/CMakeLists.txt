cmake_minimum_required(VERSION 3.8)
project(astro)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(
  DIRECTORY config description launch meshes rviz
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
    scripts/figure8_path_publisher.py
    scripts/figure8_path_follower.py
    scripts/astro_real_figure8_follower.py
    scripts/astro_real_path_publisher.py
    scripts/astar_planner.py
    scripts/astar_full_planer.py
    scripts/putanja_bag.py
    scripts/rosbag_cmdvel_replayer.py
    scripts/bag_visualizer.py
    scripts/pure_pursuit_follower.py
    scripts/astar_path_planner_dod.py
    scripts/astar_path_planner_ob.py
    scripts/cmd_safety.py
    scripts/dynamic_ob_layer.py
    scripts/rosbag_replayer.py
    scripts/zone_obstacle_detector.py
    scripts/astar_zone_planner.py
    scripts/cmd_vel_zone_gate.py
    scripts/simul_opt_bag.py
    scripts/pf_pure_pursuit.py
    scripts/astar_planner_pf.py
    scripts/astar_global_planner.py
    scripts/pfpp_local_controller.py

  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
