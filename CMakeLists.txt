cmake_minimum_required(VERSION 2.8)

add_compile_options("-W" "-Wall" "-O3" "-pedantic")
#add_compile_options("-g")

project(sfm)


find_package(Ceres REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(fmt REQUIRED)
include_directories(${CERES_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

# library
add_library(DeepArcManager src/DeepArcManager.cc)

# Make SFM executable
add_executable(sfm src/sfm.cc)
target_link_libraries(sfm  DeepArcManager ${CERES_LIBRARIES} fmt::fmt)

# Debian / Ubuntu DEB support 

install(
    TARGETS sfm
    RUNTIME DESTINATION "/usr/local/bin"
    DESTINATION "/usr/local/bin"
)

SET(CPACK_GENERATOR "DEB")
SET(CPACK_DEBIAN_PACKAGE_MAINTAINER "Vision and Learning Laboratory") 
SET(CPACK_SET_DESTDIR true)
INCLUDE(CPack)