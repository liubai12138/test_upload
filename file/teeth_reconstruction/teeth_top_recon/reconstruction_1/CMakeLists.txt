cmake_minimum_required(VERSION 3.12)
project(MyProject)

# Specify C++ standard
set(CMAKE_CXX_STANDARD 11)

# Set the path to Easy3D library
# Replace /path/to/easy3d with the actual path to your Easy3D installation or build directory
set(easy3d_DIR "E:/Easy3D/Easy3D-23fdf8957e3c1a41b51c2003136816efc4bcb91c/build")

# Request Easy3D (recommended to request only needed components)
find_package(Easy3D COMPONENTS viewer REQUIRED)

# Create an executable target
add_executable(Test main.cpp)

# Link to necessary Easy3D modules (add more if needed, e.g., algo)
target_link_libraries(Test easy3d::viewer)
