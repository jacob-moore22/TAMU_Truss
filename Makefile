CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
HDR = $(wildcard include/*.h)
TARGET = truss_solver

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

format:
	clang-format -i $(SRC) $(HDR)

clean:
	rm -f $(TARGET)
	rm -rf results

.PHONY: format clean
