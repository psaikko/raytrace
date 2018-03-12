.PHONY: all

GLFWLIBS = -lGL -lGLU -lGLEW -lglfw -lpthread
CCOPTS = -std=c++11 -Ofast -pedantic -Wall -Wextra

all:
	g++ raytrace.cpp  $(CCOPTS) $(GLFWLIBS) -o raytrace
