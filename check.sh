cd build
cmake -DCMAKE_BUILD_TYPE=Debug ..
make -s -j13
make test
lcov -d . -c -o coverage.info
lcov -r coverage.info '*/googletest/*' '*/test/*' '*/c++/*' '/usr/*' -o coverageFiltered.info
genhtml -o lcovHtml --num-spaces 4 -s --legend coverageFiltered.info
