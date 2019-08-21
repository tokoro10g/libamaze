include(GoogleTest)
 
add_executable(test_common test_common.cpp)
add_executable(test_maze test_maze.cpp)
target_link_libraries(test_common gtest)
target_link_libraries(test_maze gtest)
 
gtest_discover_tests(test_common)
gtest_discover_tests(test_maze)

add_executable(tester tester.cpp)
target_compile_options(tester PRIVATE -Wall -Wextra -Weffc++ -pedantic -Werror -pedantic-errors -Waggregate-return -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion -Wdisabled-optimization -Wfloat-equal -Wformat -Wformat=2 -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wimport -Winit-self -Winline -Winvalid-pch -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn -Wpadded -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings -Wno-error=padded -Wno-error=aggregate-return)
#add_test(IntegrationTest tester)
