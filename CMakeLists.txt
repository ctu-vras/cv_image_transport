# SPDX-License-Identifier: BSD-3-Clause
# SPDX-FileCopyrightText: Czech Technical University in Prague

cmake_minimum_required(VERSION 3.10.2)
project(cv_image_transport)

find_package(catkin REQUIRED COMPONENTS
  cras_cpp_common
  cras_topic_tools
  cv_bridge
  dynamic_reconfigure
  image_transport
  image_transport_codecs
  pluginlib
  sensor_msgs
)

catkin_python_setup()

generate_dynamic_reconfigure_options(cfg/CVPublisher.cfg)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES cv_codec
  CATKIN_DEPENDS cras_cpp_common cras_topic_tools image_transport_codecs sensor_msgs
)

include_directories(include ${catkin_INCLUDE_DIRS})

add_library(cv_codec src/cv_codec.cpp src/parse_compressed_format.cpp)
target_link_libraries(cv_codec PUBLIC ${catkin_LIBRARIES})
add_dependencies(cv_codec ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(cv_codec_plugin src/cv_codec_plugin.cpp)
target_link_libraries(cv_codec_plugin ${catkin_LIBRARIES} cv_codec)
add_dependencies(cv_codec_plugin ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

add_library(${PROJECT_NAME} src/cv_publisher.cpp src/cv_subscriber.cpp)
target_link_libraries(${PROJECT_NAME} ${catkin_LIBRARIES} cv_codec)
add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

install(TARGETS cv_codec cv_codec_plugin ${PROJECT_NAME}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.h"
)

install(FILES
  codec.xml
  transport.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

if (CATKIN_ENABLE_TESTING)
  find_package(rosbag REQUIRED)
  find_package(roslaunch REQUIRED)
  find_package(roslint REQUIRED)
  find_package(rostest REQUIRED)

  roslint_custom(catkin_lint "-W2" .)

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_FILES include/*.h src/*.c src/*.cpp test/*.cpp)
  set(ROSLINT_CPP_OPTS "--extensions=h,hpp,hh,c,cpp,cc;--linelength=120;--filter=\
    -build/header_guard,-build/include,-readability/namespace,-whitespace/braces,-runtime/references,\
    -build/c++11,-readability/nolint,-readability/todo,-legal/copyright")
  roslint_cpp(${ROSLINT_FILES})

  # Roslint C++ - checks formatting and some other rules for C++ files
  file(GLOB_RECURSE ROSLINT_PY_FILES src/*.py test/*.py)
  roslint_python(${ROSLINT_PY_FILES})

  roslint_add_test()

  include_directories(${rosbag_INCLUDE_DIRS})

  catkin_add_gtest(test_codec test/test_codec.cpp)
  target_link_libraries(test_codec ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
  add_dependencies(test_codec cv_codec_plugin)
  target_compile_definitions(test_codec PRIVATE TEST_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data\")

  catkin_add_nosetests(test/test_codec.py DEPENDENCIES cv_codec_plugin cv_codec ${catkin_EXPORTED_TARGETS})

  add_rostest_gmock(test_${PROJECT_NAME} test/test_${PROJECT_NAME}.test test/test_${PROJECT_NAME}.cpp)
  target_link_libraries(test_${PROJECT_NAME} ${catkin_LIBRARIES} ${rosbag_LIBRARIES})
  add_dependencies(test_${PROJECT_NAME} ${PROJECT_NAME})
  target_compile_definitions(test_${PROJECT_NAME} PRIVATE TEST_DATA_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/test/data\")
  roslaunch_add_file_check(test/test_${PROJECT_NAME}.test USE_TEST_DEPENDENCIES)
endif()
