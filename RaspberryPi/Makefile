# Makefile for compiling raspberry compatible
#spi_test : src/main.cpp
#	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
#all: main.o controller_engine.o js_pos_controller.o
#	g++ -g -Wall -pthread main.o -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include controller_engine.o js_pos_controller.o -o gripper

#main.o: src/main.cpp
#	g++ -c -Wall -pthread src/main.cpp -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include

controller_engine.o: src/controller_engine.cpp
	g++ -c -Wall -pthread src/controller_engine.cpp -I ./include/

js_pos_controller.o: src/js_pos_controller.cpp
	g++ -c -Wall src/js_pos_controller.cpp -I ./include/

clean:
	rm -rf *o gripper
