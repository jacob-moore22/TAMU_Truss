CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
TARGET = truss_solver

.PHONY: test clean

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

test:
	cmake -S . -B build -DBUILD_TESTS=ON
	cmake --build build -j
	ctest --test-dir build --output-on-failure

clean:
	rm -f $(TARGET)
	rm -rf results build
