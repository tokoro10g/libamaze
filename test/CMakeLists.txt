Find_package(PkgConfig)
find_package(Threads REQUIRED)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include(ExternalProject)
SET_DIRECTORY_PROPERTIES(PROPERTIES EP_PREFIX ${CMAKE_BINARY_DIR}/external)

externalproject_add(
    googletest
    URL https://github.com/google/googletest/archive/release-1.10.0.zip
    UPDATE_COMMAND ""
    INSTALL_COMMAND ""
    )

externalproject_get_property(googletest source_dir)
set(GTEST_INCLUDE_PATH ${source_dir}/googletest/include)
set(GMOCK_INCLUDE_PATH ${source_dir}/googlemock/include)

externalproject_get_property(googletest binary_dir)
set(GTEST_LIBRARY_PATH ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gtest.a) # in Unix, libgtest.a
set(GTEST_LIBRARY GTest::GTest)
add_library(${GTEST_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GTEST_LIBRARY} PROPERTIES
    IMPORTED_LOCATION ${GTEST_LIBRARY_PATH}
    INTERFACE_LINK_LIBRARIES Threads::Threads)
add_dependencies(${GTEST_LIBRARY} googletest)

set(GMOCK_LIBRARY_PATH ${binary_dir}/lib/${CMAKE_FIND_LIBRARY_PREFIXES}gmock.a) # in Unix, libgmock.a
set(GMOCK_LIBRARY GTest::GMock)
add_library(${GMOCK_LIBRARY} UNKNOWN IMPORTED)
set_target_properties(${GMOCK_LIBRARY} PROPERTIES
    IMPORTED_LOCATION ${GMOCK_LIBRARY_PATH}
    INTERFACE_LINK_LIBRARIES Threads::Threads)
add_dependencies(${GMOCK_LIBRARY} googletest)

include(GoogleTest)

add_executable(unit_test unit_test.cpp)
target_include_directories(unit_test PUBLIC ${GTEST_INCLUDE_PATH} ${GMOCK_INCLUDE_PATH})
target_link_libraries(unit_test GTest::GTest GTest::GMock --coverage)
target_compile_options(unit_test PUBLIC -Wno-error --coverage)

gtest_discover_tests(unit_test)

#target_link_libraries(tester ubsan)
#add_test(IntegrationTest tester)
