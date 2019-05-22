
# Makefile for compiling raspberry compatible
zmq_server : gripper_server.cpp
		g++  -Wall -g gripper_server.cpp -lzmq -o gripper_server `pkg-config --cflags gtk+-3.0 --libs gtk+-3.0` -lm -I ./
