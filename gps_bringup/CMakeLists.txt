cmake_minimum_required(VERSION 3.8)
project(gps_bringup)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/fix_to_path.py
  # Diagnostico del GNSS: no publican dato de dataset, sirven para saber si la
  # cadena esta rota o si simplemente falta lock del receptor.
  scripts/gnss_watchdog.py
  scripts/diagnose_gnss.sh
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # Tests del filtro de movimiento de fix_to_path (numpy-free, sin hardware).
  # Si esta linea diera problemas en tu entorno, se puede borrar junto con el
  # find_package de abajo sin afectar el build: solo apaga `colcon test`.
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_fix_to_path test/test_fix_to_path.py)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
