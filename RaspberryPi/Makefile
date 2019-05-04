# Makefile for compiling raspberry compatible
#spi_test : src/main.cpp
#	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
controller_engine.o: src/controller_engine.cpp src/finger.cpp
	g++ -Wall -pthread -g src/controller_engine.cpp src/finger.cpp -I ./include/

js_pos_controller.o: src/js_pos_controller.cpp
	g++ -Wall -g src/js_pos_controller.cpp -I/include/

clean:
	rm -rf *o
