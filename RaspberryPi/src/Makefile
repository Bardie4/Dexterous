
# Makefile for compiling raspberry compatible
spi_test : main.o
		g++ -o spi_test main.o


main.o : main.cpp
		g++ -c -lwiringPi main.cpp

clean :
		rm spi_test main.o