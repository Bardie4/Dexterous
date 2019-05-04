# Makefile for compiling raspberry compatible
#spi_test : src/main.cpp
#	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
#spi_test : main.o controller_engine.o js_pos_controller.o
#	g++ main.o controller_engine.o js_pos_controller.o -o spi_test

#main.o: src/main.cpp
#	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#
controller_engine.o: src/controller_engine.cpp
	g++ -Wall -pthread -g src/controller_engine.cpp -I ./include/

js_pos_controller.o: src/js_pos_controller.cpp
	g++ -Wall -g src/js_pos_controller.cpp -I/include/

clean:
	rm -rf *o spi_test
