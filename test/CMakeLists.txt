cmake_minimum_required(VERSION 3.5)

if(CATKIN_ENABLE_TESTING)
  foreach(
    TARGET
    test_serial_number
    )
    catkin_add_gtest(
      ${TARGET}
      ${TARGET}.cpp
      )
    target_link_libraries(
      ${TARGET}
      ${catkin_LIBRARIES}
      ${PROJECT_NAME}
      )
    add_dependencies(
      ${TARGET}
      ${${PROJECT_NAME}_EXPORTED_TARGETS}
      ${catkin_EXPORTED_TARGETS}
      )
  endforeach()
endif()
miso_add_test_headers()
