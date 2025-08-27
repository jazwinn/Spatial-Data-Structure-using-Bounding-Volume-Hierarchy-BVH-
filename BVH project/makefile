VCPKG-PATH=/vcpkg/scripts/buildsystems/vcpkg.cmake
BUILD-TYPE=Release
TEST-EXE=bin/cs350-test

.PHONY: build clean check-warnings memcheck static-analysis test

check-warnings: clean
	cmake -Bbuild ./ -DCMAKE_BUILD_TYPE=$(BUILD-TYPE) -DCMAKE_TOOLCHAIN_FILE=$(VCPKG-PATH) -DCMAKE_COMPILE_WARNING_AS_ERROR=1
	cmake --build ./build --target cs350-test

build: clean
	cmake -Bbuild ./ -DCMAKE_BUILD_TYPE=$(BUILD-TYPE) -DCMAKE_TOOLCHAIN_FILE=$(VCPKG-PATH) -DGRADING_SERVER=1
	cmake --build ./build --target cs350-test
	cmake --build ./build --target cs350-demo

memcheck: 
	valgrind --quiet --leak-check=full --leak-resolution=med --track-origins=yes --error-exitcode=-1 --vgdb=no $(TEST-EXE) --gtest_filter=*MirloRandom*
	@# No leaks but invalid accesses are detected
	@#valgrind --quiet --leak-check=no --leak-resolution=med --track-origins=yes --error-exitcode=-1 --vgdb=no $(TEST-EXE) --gtest_filter=*

test:
	$(TEST-EXE)

static-analysis:
	cmake -Bbuild ./ -DCMAKE_BUILD_TYPE=$(BUILD-TYPE) -DCMAKE_TOOLCHAIN_FILE=$(VCPKG-PATH)
	cmake --build ./build --target clang-tidy

clean:
	rm -rf build/