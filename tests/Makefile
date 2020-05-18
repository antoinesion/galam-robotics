CXX=g++
CXXFLAGS=-std=c++11
EXE=test
OBJS=$(patsubst %.cpp,%.o,$(wildcard *.cpp))

all: $(EXE)

$(EXE): $(OBJS)
	$(CXX) -g $^ -o $@

%.o : %.cpp
	$(CXX) -g $(CXXFLAGS) -c $< -o $@

clean:
	rm $(OBJS)
	rm $(EXE)
