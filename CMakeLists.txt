cmake_minimum_required (VERSION 2.6)
project (LibAmaze)

include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(include)
enable_testing()
add_subdirectory(test)
