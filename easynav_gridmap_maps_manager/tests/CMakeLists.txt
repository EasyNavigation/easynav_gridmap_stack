ament_add_gtest(gridmap_maps_builder_tests gridmap_maps_builder_tests.cpp)
target_include_directories(gridmap_maps_builder_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gridmap_maps_builder_tests
  ${PROJECT_NAME}  ${dependencies}
)

ament_add_gtest(gridmap_maps_manager_tests gridmap_maps_manager_tests.cpp)
target_include_directories(gridmap_maps_manager_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gridmap_maps_manager_tests
  ${PROJECT_NAME}  ${dependencies}
)

ament_add_gtest(gridmap_utils_tests gridmap_utils_tests.cpp)
target_include_directories(gridmap_utils_tests PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(gridmap_utils_tests
  ${PROJECT_NAME} ${dependencies}
)
