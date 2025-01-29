.PHONY: clean build run_tests

clean:
	rm -r ./build

build:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Release -DCMAKE_EXPORT_COMPILE_COMMANDS=1 -DCMAKE_C_FLAGS_RELEASE="-O3" \
        -DCMAKE_CXX_FLAGS_RELEASE="-march=native -mtune=native -O3 -no-pie" -DCMAKE_CXX_FLAGS="-march=native -mtune=native -O3 -no-pie" 
	cmake --build build --config Release

build_debug:
	mkdir -p build
	cmake -B build -DCMAKE_BUILD_TYPE=Debug -DCMAKE_EXPORT_COMPILE_COMMANDS=1
	cd ./build
	make -j32
	cd ../

run_tests:
	if [ -d build/tests ]; then ctest --test-dir build/tests; fi
