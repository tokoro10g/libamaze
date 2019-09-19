find_package(PkgConfig)
pkg_check_modules(GTEST REQUIRED gtest>=1.7.0)

include(GoogleTest)
include_directories(
    ${GTEST_INCLUDE_DIRS}
    )
 
add_executable(test_common test_common.cpp)
add_executable(test_maze test_maze.cpp)
add_executable(test_mazegraph test_mazegraph.cpp)
target_link_libraries(test_common gtest)
target_link_libraries(test_maze gtest)
target_link_libraries(test_mazegraph gtest)
 
gtest_discover_tests(test_common)
gtest_discover_tests(test_maze)
gtest_discover_tests(test_mazegraph)

add_executable(tester tester.cpp)
#target_link_libraries(tester ubsan)
#add_test(IntegrationTest tester)
