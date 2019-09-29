cmake_minimum_required (VERSION 3.7)
project (LibAmaze)

include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CTEST_OUTPUT_ON_FAILURE 1)

include_directories(
    include
    )

enable_testing()

add_compile_options(-Wfatal-errors -Wall -Wextra -Weffc++ -pedantic -Werror -pedantic-errors -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion -Wdisabled-optimization -Wfloat-equal -Wformat -Wformat=2 -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wimport -Winit-self -Winvalid-pch -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings -funroll-loops -ffast-math)

add_subdirectory(test)
add_subdirectory(samples)
