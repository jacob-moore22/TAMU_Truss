CXX = g++
CXXFLAGS = -std=c++17 -O2 -Iinclude

SRC = $(wildcard src/*.cpp)
TARGET = truss_solver

$(TARGET): $(SRC)
	$(CXX) $(CXXFLAGS) $(SRC) -o $(TARGET)

clean:
	rm -f $(TARGET)
	rm -rf results
