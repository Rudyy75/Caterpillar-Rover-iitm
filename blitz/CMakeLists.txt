cmake_minimum_required(VERSION 3.8)
project(blitz)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
# find_package(<dependency> REQUIRED)

install(
  DIRECTORY
    blitz
    demo
    launch
  DESTINATION
    share/${PROJECT_NAME}/
)

install(PROGRAMS
    blitz/packer.py
    blitz/parser.py
    blitz/interfaces.py
    blitz/blitz.py
    demo/blitz_demo.py

    DESTINATION lib/${PROJECT_NAME}
)
ament_package()