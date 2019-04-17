# Makefile for compiling raspberry compatible
spi_test : main.cpp
	g++ -Wall -pthread -g main.cpp -o spi_test -lzmq -lpthread -lpigpio -lrt -lm -I/Dexterous/RaspberryPi/include  -L/usr/local/lib -lwiringPi -I/usr/local/include
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
