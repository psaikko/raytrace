.PHONY: all

GLFWDIR = glfw
GLFWLIBS = -lGL -lGLU -lGLEW -lglfw3 -lX11 -lXxf86vm -lXrandr -lpthread -lXi -lXcursor -lXinerama
CCOPTS = -std=c++11 -Ofast -pedantic -Wall -Wextra

all:
	g++ raytrace.cpp -I$(GLFWDIR)/include -L$(GLFWDIR)/lib $(CCOPTS) $(GLFWLIBS) -o raytrace
