cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

SET(sampleName RA_Calculator)

add_executable (RA_Calculator RA_Calculator.cpp Joints_Solvers.cpp Joints_Solvers.h RobotArm.cpp RobotArm.h stdafx.cpp stdafx.h)