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
add_subdirectory(test)
add_subdirectory(samples)
