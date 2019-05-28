# Makefile for compiling raspberry compatible
#spi_test : src/main.cpp
#	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
#all: main.o controller_engine.o js_pos_controller.o
#	g++ -g -Wall -pthread main.o -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include controller_engine.o js_pos_controller.o -o gripper

all: gripper_monitor.o
	g++ -g -Wall -pthread gripper_monitor.o -o gripper_monitor -lzmq -lpthread  -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -I/usr/include/python2.7 -lpython2.7

gripper_monitor.o: gripper_monitor.cpp
	g++ -c -Wall -pthread gripper_monitor.cpp -lzmq -lpthread -lrt -lm -I ./include/ -I ./lib/ -L/usr/local/lib -I/usr/include/python2.7 -lpython2.7
