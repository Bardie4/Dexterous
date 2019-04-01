# Makefile for compiling raspberry compatible
spi_test : main.c
	gcc -Wall -pthread -g main.c -o spi_test -lzmq -lpthread -lpigpio -lrt -lm
#gcc -Wall -g main.c -lzmq -o spi_test -lpigpio -lpthread -pthread
