cd build
make -s -j13
make test
cd test/CMakeFiles/unit_test.dir/
lcov -d . -c -o coverage.info
lcov -r coverage.info '*/googletest/*' '*/test/*' '*/c++/*' '/usr/*' -o coverageFiltered.info
genhtml -o lcovHtml --num-spaces 4 -s --legend coverageFiltered.info
