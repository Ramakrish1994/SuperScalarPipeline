# Makefile
#
SH = bash
CFLAGS = -g
LDFLAGS = -lm

all: main

main: simulator.o main.o
	g++ $(LDFLAGS) $^ -o $@

main.o: main.cpp simulator.h
	g++ -c $(CFLAGS) $< -o $@

simulator.o: simulator.cpp simulator.h
	g++ -c $(CFLAGS) $< -o $@

clean:
	rm -rf *.o main

