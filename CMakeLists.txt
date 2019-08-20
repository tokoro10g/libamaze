cmake_minimum_required (VERSION 2.6)
project (LibAmaze)

include(CTest)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CTEST_OUTPUT_ON_FAILURE 1)

find_package(PkgConfig)
pkg_check_modules(GTEST REQUIRED gtest>=1.7.0)

include_directories(
    include
    ${GTEST_INCLUDE_DIRS}
    )

enable_testing()
add_subdirectory(test)
