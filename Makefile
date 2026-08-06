CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
TARGET = truss_solver

.PHONY: test docs clean

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test:
	cmake -S . -B build -DBUILD_TESTS=ON
	cmake --build build -j
	ctest --test-dir build --output-on-failure

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
