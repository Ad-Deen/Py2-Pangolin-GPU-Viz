cmake_minimum_required(VERSION 3.10)
project(PangoVisualizer)

# 1. Point to your manual Pangolin build
# Change this path if your build folder is named differently (e.g., /home/deen/Pangolin/build)
set(Pangolin_DIR "/home/deen/Pangolin/build")

# 2. Find Dependencies
find_package(Pangolin REQUIRED)
find_package(Eigen3 REQUIRED)

# 3. Create the executable
add_executable(pango_viz pango_viz.cpp)
add_executable(viz_util viz_util.cpp)

# 4. Link the libraries
target_link_libraries(pango_viz 
    PUBLIC 
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen
)
target_link_libraries(viz_util 
    PUBLIC 
    ${Pangolin_LIBRARIES}
    Eigen3::Eigen
)