RA_Calculator: RA_Calculator.o RobotArm.o stdafx.o Joints_Solvers.o
	g++ RA_Calculator.o RobotArm.o stdafx.o Joints_Solvers.o -o RA_Calculator

RA_Calculator.o: RA_Calculator.cpp
	g++ -std=c++0x -c  RA_Calculator.cpp

RobotArm.o: RobotArm.cpp RobotArm.h
	g++ -std=c++0x -c  RobotArm.cpp

stdafx.o: stdafx.cpp stdafx.h
	g++ -std=c++0x -c  stdafx.cpp

Joints_Solver.o: Joints_Solvers.cpp Joints_Solvers.h
	g++ -std=c++0x -c  Joints_Solvers.cpp

clean:
	rm *.o RA_Calculator