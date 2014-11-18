OMPL_DIR = /usr
CXXFLAGS = -g # change to -g when debugging code # -02
INCLUDE_FLAGS = -I${OMPL_DIR}/include
LD_FLAGS = -L${OMPL_DIR}/lib -lompl -lompl_app -lboost_program_options -lboost_system
CXX=c++
#JointManipulatorSpace.o
#JointManipulatorSpace.o 
PointRobot: planner.o 
	$(CXX) $(CXXFLAGS) -o Planner planner.o $(LD_FLAGS)

clean:
	rm *.o

%.o: %.cpp
	$(CXX) -c $(CXXFLAGS) $(INCLUDE_FLAGS) $< -o $@
