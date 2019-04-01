
# Makefile for compiling raspberry compatible
zmq_server : gripper_server.c
		gcc -Wall -g gripper_server.c -lzmq -o gripper_server `pkg-config --cflags gtk+-2.0 --libs gtk+-2.0`
