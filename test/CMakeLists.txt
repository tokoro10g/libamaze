include(GoogleTest)
 
add_executable(test_common test_common.cpp)
add_executable(test_maze test_maze.cpp)
target_link_libraries(test_common gtest)
target_link_libraries(test_maze gtest)
 
gtest_discover_tests(test_common)
gtest_discover_tests(test_maze)

add_executable(tester tester.cpp)
#add_test(IntegrationTest tester)
