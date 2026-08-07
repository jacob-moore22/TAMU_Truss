CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
TARGET = truss_solver
FMT_FILES = $(wildcard include/*.h src/*.cpp tests/*.cpp)

.PHONY: test docs format format-check clean

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test:
	cmake -S . -B build -DBUILD_TESTS=ON
	cmake --build build -j
	ctest --test-dir build --output-on-failure

format:
	clang-format -i $(FMT_FILES)

format-check:
	clang-format --dry-run --Werror $(FMT_FILES)

docs:
	doxygen Doxyfile
	rm -rf site
	mkdir -p site
	cp -r doxygen-build/html site/doxygen
	cp docs/index.html site/index.html
	@echo "Open site/index.html in your browser to preview (matches the deployed Pages layout)."

clean:
	rm -f $(TARGET)
	rm -rf results build doxygen-build site
