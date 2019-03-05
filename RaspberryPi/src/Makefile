
# Makefile for compiling raspberry compatible



SOURCES = main.cpp wiringPiSPI.h

OBJECTS = $(SOURCES:%.c=%.o)

COMPILER =  gcc



TARGET = main





$(TARGET): $(OBJECTS)

	$(COMPILER) $^ -o $@



%.o: %.c

	$(COMPILER) -c $< -o $@



copy:

	scp $(TARGET) pi@10.218.130.162:~/test



clean:

	rm $(TARGET)