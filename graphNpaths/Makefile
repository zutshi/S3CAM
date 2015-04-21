ANN_LIBS=./ann_1.1.2/lib/
ANN_INCLUDES=./ann_1.1.2/include/

CFLAGS=-g -Wall -O3
LFLAGS= -lcurses  -L $(ANN_LIBS) -lANN 
IFLAGS=-I $(ANN_INCLUDES) 

OBJS = ./sparseMat.o ./readWriteGraph.o ./createGraph.o

all: createGraph

progBarTest: progBar.o progBarTest.cpp
	g++ -o $@ $^ $(CFLAGS) -lcurses

createGraph: $(OBJS) progBar.o
	g++ -o $@ $^ $(CFLAGS) $(LFLAGS) $(IFLAGS)

%.o:%.cpp
	g++ -o $@  $(CFLAGS) $(IFLAGS) -c $<

progBar.o: progBar.cpp
	g++ -o $@ -c $<

clean: 
	rm -f $(OBJS) ./progBar.o ./createGraph
