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
target_compile_options(tester PRIVATE -Wfatal-errors -Wall -Wextra -Weffc++ -pedantic -Werror -pedantic-errors -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion -Wdisabled-optimization -Wfloat-equal -Wformat -Wformat=2 -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wimport -Winit-self -Winline -Winvalid-pch -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings)
#target_link_libraries(tester ubsan)
#add_test(IntegrationTest tester)
