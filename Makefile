CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
HDR = $(wildcard include/*.h)
TARGET = truss_solver

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

format:
	clang-format -i $(SRC) $(HDR)

docs:
	doxygen Doxyfile

clean:
	rm -f $(TARGET)
	rm -rf results

clean-docs:
	rm -rf docs

.PHONY: format docs clean clean-docs
