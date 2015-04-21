CFLAGS=-g -Wall -O3

all: ktest

ktest:  ./2heap.h ./graph.h ./kbest.h ./ktest.cc ./list.h
	g++ $(CFLAGS) ./ktest.cc -o ./ktest

clean: 
	rm ./ktest
