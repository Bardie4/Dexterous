# Makefile for compiling raspberry compatible

spi_test : src/main.cpp
	g++ -Wall -pthread -g src/main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I ./include/ -I ./lib/  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
