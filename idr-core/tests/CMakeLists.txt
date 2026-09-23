add_executable(test_math_so3 test_math_so3.cpp)
target_link_libraries(test_math_so3 PRIVATE idr_core)
add_test(NAME test_math_so3 COMMAND test_math_so3)

add_executable(test_math_golden_parity test_math_golden_parity.cpp)
target_link_libraries(test_math_golden_parity PRIVATE idr_core)
add_test(NAME test_math_golden_parity COMMAND test_math_golden_parity)

add_executable(test_gbdt_parity test_gbdt_parity.cpp)
target_link_libraries(test_gbdt_parity PRIVATE idr_core)
add_test(NAME test_gbdt_parity COMMAND test_gbdt_parity)
set_tests_properties(test_gbdt_parity PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/..")

add_executable(test_feature_vibration test_feature_vibration.cpp)
target_link_libraries(test_feature_vibration PRIVATE idr_core)
add_test(NAME test_feature_vibration COMMAND test_feature_vibration)
set_tests_properties(test_feature_vibration PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/..")

add_executable(test_alignment test_alignment.cpp)
target_link_libraries(test_alignment PRIVATE idr_core)
add_test(NAME test_alignment COMMAND test_alignment)

add_executable(test_eskf test_eskf.cpp)
target_link_libraries(test_eskf PRIVATE idr_core)
add_test(NAME test_eskf COMMAND test_eskf)

add_executable(test_heading_supervisor test_heading_supervisor.cpp)
target_link_libraries(test_heading_supervisor PRIVATE idr_core)
add_test(NAME test_heading_supervisor COMMAND test_heading_supervisor)

add_executable(test_map_matcher test_map_matcher.cpp)
target_link_libraries(test_map_matcher PRIVATE idr_core)
add_test(NAME test_map_matcher COMMAND test_map_matcher)

add_executable(test_navigation_engine test_navigation_engine.cpp)
target_link_libraries(test_navigation_engine PRIVATE idr_core)
add_test(NAME test_navigation_engine COMMAND test_navigation_engine)

add_executable(test_c_api test_c_api.cpp)
target_link_libraries(test_c_api PRIVATE idr_core)
add_test(NAME test_c_api COMMAND test_c_api)

add_executable(test_idrmap_reader test_idrmap_reader.cpp)
target_link_libraries(test_idrmap_reader PRIVATE idr_core)
add_test(NAME test_idrmap_reader COMMAND test_idrmap_reader)
set_tests_properties(test_idrmap_reader PROPERTIES WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}/..")

add_executable(benchmark_tactical_100k benchmark_tactical_100k.cpp)
target_link_libraries(benchmark_tactical_100k PRIVATE idr_core)
