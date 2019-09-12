add_executable(fourway_static fourway_static.cpp)
add_executable(fourway_dynamic fourway_dynamic.cpp)
add_executable(sixway_static sixway_static.cpp)
add_executable(sixway_dynamic sixway_dynamic.cpp)
add_executable(unknown_32 unknown_32.cpp)
 
add_compile_options(-Wfatal-errors -Wall -Wextra -Weffc++ -pedantic -Werror -pedantic-errors -Wcast-align -Wcast-qual -Wchar-subscripts -Wcomment -Wconversion -Wdisabled-optimization -Wfloat-equal -Wformat -Wformat=2 -Wformat-nonliteral -Wformat-security -Wformat-y2k -Wimport -Winit-self -Winline -Winvalid-pch -Wunsafe-loop-optimizations -Wlong-long -Wmissing-braces -Wmissing-field-initializers -Wmissing-format-attribute -Wmissing-include-dirs -Wmissing-noreturn -Wparentheses -Wpointer-arith -Wredundant-decls -Wreturn-type -Wsequence-point -Wsign-compare -Wstack-protector -Wstrict-aliasing -Wstrict-aliasing=2 -Wswitch -Wswitch-default -Wswitch-enum -Wtrigraphs -Wuninitialized -Wunknown-pragmas -Wunreachable-code -Wunused -Wunused-function -Wunused-label -Wunused-parameter -Wunused-value -Wunused-variable -Wvariadic-macros -Wvolatile-register-var -Wwrite-strings)
