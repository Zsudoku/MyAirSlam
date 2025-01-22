cmake_minimum_required( VERSION 2.8 )
project( joinMap )

set( CMAKE_BUILD_TYPE Release )
set( CMAKE_CXX_FLAGS "-std=c++11 -O3" )
set(CMAKE_CXX_STANDARD 14)
# opencv 
find_package( OpenCV REQUIRED )
include_directories( ${OpenCV_INCLUDE_DIRS} )

# eigen 
include_directories( "/usr/include/eigen3/" )

# pcl 
find_package( PCL REQUIRED COMPONENT common io filters)
include_directories( ${PCL_INCLUDE_DIRS} )
add_definitions( ${PCL_DEFINITIONS} )

add_executable( joinMap joinMap.cpp )
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

# add_executable( plyobj ply_obj.cpp )
# target_link_libraries( plyobj ${Boost_LIBRARIES} ${PCL_LIBRARIES} )

# add_executable( putimg put_img.cpp )
# target_link_libraries( putimg ${Boost_LIBRARIES} ${PCL_LIBRARIES} )