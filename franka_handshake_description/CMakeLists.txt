cmake_minimum_required(VERSION 3.8)
project(franka_handshake_description)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY config launch meshes rviz urdf test include src robots
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
find_package(ament_cmake_pytest REQUIRED)

  ament_add_pytest_test(test_franka_handshake_urdf_xacro test/franka_handshake_test_urdf_xacro.py)
endif()

ament_package()
